// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 ASPEED Technology Inc.
 */
#include <linux/bug.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/mtd/mtd.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#include <linux/mfd/aspeed-espi.h>

#define DRV_NAME  "aspeed-espi-mafs"
#define MAFS_TIMEOUT_MS msecs_to_jiffies(100)
#define MAFS_BOOT_TIMEOUT_MS msecs_to_jiffies(10000)

/*
 * eSPI cycle type encoding
 *
 * Section 5.1 Cycle Types and Packet Format,
 * Intel eSPI Interface Base Specification, Rev 1.0, Jan. 2016.
 */
 #define ESPI_FLASH_READ		0x00
 #define ESPI_FLASH_WRITE		0x01
 #define ESPI_FLASH_ERASE		0x02
 #define ESPI_FLASH_SUC_CMPLT		0x06
 #define ESPI_FLASH_SUC_CMPLT_D_MIDDLE	0x09
 #define ESPI_FLASH_SUC_CMPLT_D_FIRST	0x0b
 #define ESPI_FLASH_SUC_CMPLT_D_LAST	0x0d
 #define ESPI_FLASH_SUC_CMPLT_D_ONLY	0x0f
 #define ESPI_FLASH_UNSUC_CMPLT		0x0c
 #define ESPI_FLASH_UNSUC_CMPLT_ONLY	0x0e

struct aspeed_espi_flash_dma {
	void *tx_virt;
	dma_addr_t tx_addr;
	void *rx_virt;
	dma_addr_t rx_addr;
};

/**
 * struct aspeed_espi_flash - driver data
 * @lock:             Prevents concurrent access to the HW
 * @dma:              DMA buffers used for RX and TX
 * @mtd:              mtd info structure
 * @regmap:           Device's regmap. Only direct access registers.
 * @erase_mask:       bitmask encoding erase types to use when sending
 *                    an ERASE command
 * @tx_sts:           TX status set on IRQ
 * @rx_sts:           RX status set on IRQ
 * @max_payload_size: Maximum eSPI flash channel payload size
 * @wq:               Wait queue for thread work
 * @spinlock:         Spinlock protects rx_sts and tx_sts
 * @irq_rx_complt:    Linux IRQ number for flash RX completion
 * @irq_tx_complt:    Linux IRQ number for flash TX completion
 * @irq_rx_abt:       Linux IRQ number for flash RX abort
 * @irq_tx_abt:       Linux IRQ number for flash TX abort
 * @irq_tx_err:       Linux IRQ number for flash TX error
 */
struct aspeed_espi_flash {
	struct mutex			lock;
	struct aspeed_espi_flash_dma	dma;
	struct mtd_info			mtd;
	struct regmap			*regmap;
	uint8_t				erase_mask;
	uint32_t			tx_sts;
	uint32_t			rx_sts;
	size_t				max_payload_size;
	wait_queue_head_t		wq;
	spinlock_t			spinlock;
	int				irq_rx_complt;
	int				irq_tx_complt;
	int				irq_rx_abt;
	int				irq_tx_abt;
	int				irq_tx_err;
};

/**
 * aspeed_espi_ready - Returns true when eSPI MAFS can be used
 *
 * @dev: device to check
 * @timeout: timeout value in jiffies
 */
static int aspeed_espi_mafs_ready(struct device *dev, unsigned long timeout)
{
	struct aspeed_espi_flash *espi_flash = dev_get_drvdata(dev);
	u32 cap_reg;

	timeout += jiffies;
	/* Check if HW is ready. */
	do {
		/* Test if VW channel is ready */
		regmap_read(espi_flash->regmap, ESPI_CH3_CAP_N_CONF, &cap_reg);
		if ((cap_reg & (ESPI_CH3_CAP_N_READY | ESPI_CH3_CAP_N_ENABLED)) ==
		    (ESPI_CH3_CAP_N_READY | ESPI_CH3_CAP_N_ENABLED))
			return 0;

		msleep(1);
	} while (time_before(jiffies, timeout));

	dev_err(dev, "timed out waiting for HW ready\n");

	return -ETIMEDOUT;
}

/**
 * aspeed_espi_flash_rx - RX an eSPI packet over the flash channel
 * @mtd: mtd info structure
 * @cyc: Received eSPI packet cycle type
 * @len: Received eSPI packet length

 * Receives an eSPI packet over the flash channel or waits until an error
 * had occured or the request timed out. Fills out the cycle type and length.
 */
static long aspeed_espi_flash_rx(struct mtd_info *mtd, uint8_t *cyc, uint16_t *len)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	unsigned long flags;
	u32 reg;
	int ret;

	/* Test if flash channel is ready */
	ret = aspeed_espi_mafs_ready(mtd->dev.parent, MAFS_TIMEOUT_MS);
	if (ret)
		return ret;

	spin_lock_irqsave(&espi_flash->spinlock, flags);
	ret = wait_event_interruptible_lock_irq_timeout(espi_flash->wq,
						        espi_flash->rx_sts,
						        espi_flash->spinlock,
						        MAFS_TIMEOUT_MS);

	spin_unlock_irqrestore(&espi_flash->spinlock, flags);

	if (ret == -ERESTARTSYS)
		return -EINTR;
	else if (!ret)
		return -ETIMEDOUT;
	else if (espi_flash->rx_sts & ESPI_INT_STS_FLASH_RX_ABT)
		return -EFAULT;

	/* common header (i.e. cycle type, tag, and length) is taken by HW */
	regmap_read(espi_flash->regmap, ESPI_FLASH_RX_CTRL, &reg);
	if (cyc)
		*cyc = (reg & ESPI_FLASH_RX_CTRL_CYC_MASK) >> ESPI_FLASH_RX_CTRL_CYC_SHIFT;
	if (len)
		*len = (reg & ESPI_FLASH_RX_CTRL_LEN_MASK) >> ESPI_FLASH_RX_CTRL_LEN_SHIFT;

	return 0;
}

/**
 * aspeed_espi_flash_rx_ack - Tell HW that RX packet has been processed
 * @mtd: mtd info structure
 *
 * Acknowledges the reception of a packet.
 */
static void aspeed_espi_flash_rx_ack(struct mtd_info *mtd)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	unsigned long flags;

	spin_lock_irqsave(&espi_flash->spinlock, flags);

	/* Signal that packet has been serviced */
	regmap_set_bits(espi_flash->regmap, ESPI_FLASH_RX_CTRL,
			ESPI_FLASH_RX_CTRL_PEND_SERV);

	espi_flash->rx_sts = 0;

	spin_unlock_irqrestore(&espi_flash->spinlock, flags);
}

/**
 * aspeed_espi_flash_rx_data - RX an eSPI packet over the flash channel with data
 * @mtd: mtd info structure
 * @cyc: Received eSPI packet cycle type
 * @len: Received eSPI packet length
 * @addr: Received eSPI packet address
 * @data: Pointer to a buffer to write the eSPI packet payload to
 * @data_len: Size of the buffer to write the eSPI packet payload to
 *
 * Receives an eSPI packet over the flash channel that contains data or waits
 * until an error had occured or the request timed out.
 */
static long aspeed_espi_flash_rx_data(struct mtd_info *mtd, uint8_t *cyc, uint16_t *len,
				      uint32_t *addr, uint8_t *data, size_t *data_len)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	uint32_t rx_pkt_len;
	void *rx_virt_ptr;
	int ret;

	if (!mtd || !cyc || !len || !data || !data_len || !*data_len)
		return -EINVAL;

	/* Honour maximum payload size */
	if (*data_len > espi_flash->max_payload_size)
		return -EINVAL;

	ret = aspeed_espi_flash_rx(mtd, cyc, len);
	if (ret) {
		aspeed_espi_flash_rx_ack(mtd);
		return ret;
	}

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	switch (*cyc) {
	case ESPI_FLASH_SUC_CMPLT_D_MIDDLE:
	case ESPI_FLASH_SUC_CMPLT_D_FIRST:
	case ESPI_FLASH_SUC_CMPLT_D_LAST:
	case ESPI_FLASH_SUC_CMPLT_D_ONLY:
		rx_pkt_len = (*len) ? *len : PAGE_SIZE;
		rx_virt_ptr = espi_flash->dma.rx_virt;
		break;
	case ESPI_FLASH_SUC_CMPLT:
	case ESPI_FLASH_UNSUC_CMPLT:
	case ESPI_FLASH_UNSUC_CMPLT_ONLY:
		/* No data received */
		rx_pkt_len = 0;
		rx_virt_ptr = NULL;
		break;
	default:
		rx_pkt_len = 0;
		ret = -EFAULT;
	}
	if (data_len)
		*data_len = rx_pkt_len;

	if (rx_pkt_len && data_len && data) {
		if (*data_len >= rx_pkt_len)
			memcpy(data, rx_virt_ptr, rx_pkt_len);
		else
			ret = -EINVAL;
	}

	aspeed_espi_flash_rx_ack(mtd);

	return ret;
}

/**
 * aspeed_espi_flash_rx_get_completion - Wait for successful completion
 * @mtd: mtd info structure
 * @cyc: Received eSPI packet cycle type
 * @len: Received eSPI packet length
 * @addr: Received eSPI packet address
 * @data: Pointer to a buffer to write the eSPI packet payload to
 * @data_len: Size of the buffer to write the eSPI packet payload to
 *
 * Waits to receives an eSPI packet over the flash channel that signals
 * successful completion of the previous transfered eSPI packet. Typically
 * used after a WRITE or ERASE flash command.
 */
static long aspeed_espi_flash_rx_get_completion(struct mtd_info *mtd)
{
	struct device *dev = &mtd->dev;
	uint8_t cyc;
	int ret;

	/* Test if flash channel is ready */
	ret = aspeed_espi_mafs_ready(mtd->dev.parent, MAFS_TIMEOUT_MS);
	if (ret)
		return ret;

	ret = aspeed_espi_flash_rx(mtd, &cyc, NULL);
	aspeed_espi_flash_rx_ack(mtd);

	if (ret)
		return ret;

	if (cyc != ESPI_FLASH_SUC_CMPLT) {
		dev_info(dev, "Rx response was not successful\n");
		return -EFAULT;
	}
	return 0;
}

/**
 * aspeed_espi_flash_put_tx - TX an eSPI packet over the flash channel
 * @mtd: mtd info structure
 * @cyc: eSPI packet cycle type
 * @len: eSPI packet length
 * @addr: eSPI packet address
 * @data: eSPI packet payload data
 * @data_len: eSPI packet payload data length
 *
 * Transmits an eSPI packet over the flash channel and waits until the
 * packet has been transfered or an error occured.
 *
 */
static long aspeed_espi_flash_put_tx(struct mtd_info *mtd, uint8_t cyc, uint16_t len,
				     const uint32_t addr, const uint8_t *data, size_t data_len)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	struct device *dev = &mtd->dev;
	unsigned long flags;
	uint32_t reg, addr_be;
	int ret;

	/* Honour maximum payload size */
	if (data_len > espi_flash->max_payload_size)
		return -EINVAL;

	/* Test if flash channel is ready */
	ret = aspeed_espi_mafs_ready(mtd->dev.parent, MAFS_TIMEOUT_MS);
	if (ret)
		return ret;

	/*
	 * Test if TX ready. It should never be busy as the code is half duplex
	 * and single threaded.
	 */
	regmap_read(espi_flash->regmap, ESPI_FLASH_TX_CTRL, &reg);
	if (unlikely(reg & ESPI_FLASH_TX_CTRL_TRIGGER)) {
		dev_err(dev, "Tx operation still pending\n");
		return -EBUSY;
	}

	/*
	 * Packet format:
	 * - common header (i.e. cycle type, tag, and length)
	 *   part is written to HW registers
	 * - Address and payload is written to DMA buffer
	 */
	addr_be = htonl(addr);
	memcpy(espi_flash->dma.tx_virt, &addr_be, sizeof(addr_be));
	if (data)
		memcpy(espi_flash->dma.tx_virt + sizeof(addr_be), data, data_len);
	dma_wmb();

	spin_lock_irqsave(&espi_flash->spinlock, flags);
	espi_flash->tx_sts = 0;
	espi_flash->rx_sts = 0;

	reg = ((cyc << ESPI_FLASH_TX_CTRL_CYC_SHIFT) & ESPI_FLASH_TX_CTRL_CYC_MASK)
		| ((len << ESPI_FLASH_TX_CTRL_LEN_SHIFT) & ESPI_FLASH_TX_CTRL_LEN_MASK)
		| ESPI_FLASH_TX_CTRL_TRIGGER;

	/* Signal that packet is ready for TX */
	regmap_write(espi_flash->regmap, ESPI_FLASH_TX_CTRL, reg);

	ret = wait_event_interruptible_lock_irq_timeout(espi_flash->wq,
						        espi_flash->tx_sts,
						        espi_flash->spinlock,
						        MAFS_TIMEOUT_MS);
	spin_unlock_irqrestore(&espi_flash->spinlock, flags);

	if (ret == -ERESTARTSYS)
		ret = -EINTR;
	else if (!ret) {
		dev_info(dev, "Tx operation not processed within timeout\n");
		ret = -ETIMEDOUT;
	} else if (espi_flash->tx_sts & (ESPI_INT_STS_FLASH_TX_ERR | ESPI_INT_STS_FLASH_TX_ABT)) {
		dev_info(dev, "Tx operation declined by remote, TX_CTRL=0x%x\n", reg);
		ret = -EFAULT;
	} else
		ret = 0;

	return ret;
}

/**
 * aspeed_espi_flash_erase - Sends an ERASE eSPI packet
 * @mtd: mtd info structure
 * @instr: erase instruction
 *
 * Sends out ERASE eSPI packets over the flash channel and waits for
 * successful completion. The length and address must be aligned to
 * the advertised erasesize.
 */
static int aspeed_espi_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	int ret = 0;

	/* Sanity checks */
	if ((uint32_t)instr->len % espi_flash->mtd.erasesize)
		return -EINVAL;

	if ((uint32_t)instr->addr % espi_flash->mtd.erasesize)
		return -EINVAL;

	mutex_lock(&espi_flash->lock);

	while (instr->len) {
		ret = aspeed_espi_flash_put_tx(mtd, ESPI_FLASH_ERASE,
					       espi_flash->erase_mask, instr->addr,
					       NULL, 0);
		if (ret)
			goto unlock_mtx_n_out;

		ret = aspeed_espi_flash_rx_get_completion(mtd);
		if (ret)
			goto unlock_mtx_n_out;

		instr->len -= espi_flash->mtd.erasesize;
		instr->addr += espi_flash->mtd.erasesize;
	}

unlock_mtx_n_out:
	instr->fail_addr = instr->addr;

	mutex_unlock(&espi_flash->lock);
	return ret;
}

/**
 * aspeed_espi_flash_read - Sends an READ eSPI packet
 * @mtd: MTD device description object
 * @from: absolute offset from where to read
 * @len: how many bytes to read
 * @retlen: count of read bytes is returned here
 * @buf: buffer to store the read data
 *
 * Sends out READ eSPI packets over the flash channel and waits for
 * successful completion with data.
 */
static int aspeed_espi_flash_read(struct mtd_info *mtd, loff_t from, size_t len,
				  size_t * retlen, u_char * buf)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	struct device *dev = &mtd->dev;
	uint16_t len_pt, len_rx;
	size_t pkt_len;
	uint8_t cyc;
	int ret = 0;

	mutex_lock(&espi_flash->lock);

	while (len) {
		len_pt = min(len, espi_flash->max_payload_size);

		ret = aspeed_espi_flash_put_tx(mtd, ESPI_FLASH_READ, len_pt, from, NULL, 0);
		if (ret)
			goto unlock_mtx_n_out;

		pkt_len = len_pt;
		len_rx = 0;
		cyc = 0;
		ret = aspeed_espi_flash_rx_data(mtd, &cyc, &len_rx, NULL, buf, &pkt_len);
		if (ret)
			goto unlock_mtx_n_out;

		if (cyc != ESPI_FLASH_SUC_CMPLT_D_ONLY) {
			dev_err(dev, "Rx response was not successful\n");
			ret = -EFAULT;
			goto unlock_mtx_n_out;
		}
		if (pkt_len != len_pt) {
			dev_err(dev, "Rx response has unexpected data length\n");
			ret = -ENODATA;
			goto unlock_mtx_n_out;
		}

		len -= min(len_pt, len);
		buf += len_pt;
		from += len_pt;
		*retlen += len_pt;
	}

unlock_mtx_n_out:
	mutex_unlock(&espi_flash->lock);
	return ret;
}

/**
 * aspeed_espi_flash_write - Sends an WRITE eSPI packet
 * @mtd: MTD device description object
 * @from: absolute offset from where to read
 * @len: how many bytes to read
 * @retlen: count of read bytes is returned here
 * @buf: buffer to store the read data
 *
 * Sends out WRITE eSPI packets over the flash channel and waits for
 * successful completion.
 */
static int aspeed_espi_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
				   size_t * retlen, const u_char * buf)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	uint16_t len_pt;
	int ret = 0;

	mutex_lock(&espi_flash->lock);

	while (len) {
		len_pt = min(len, espi_flash->max_payload_size);

		ret = aspeed_espi_flash_put_tx(mtd, ESPI_FLASH_WRITE, len_pt, to, buf, len_pt);
		if (ret)
			goto unlock_mtx_n_out;

		ret = aspeed_espi_flash_rx_get_completion(mtd);
		if (ret)
			goto unlock_mtx_n_out;

		len -= len_pt;
		buf += len_pt;
		to += len_pt;
		*retlen += len_pt;
	}

unlock_mtx_n_out:
	mutex_unlock(&espi_flash->lock);
	return ret;
}

/**
 * aspeed_espi_flash_irq - Shared IRQ handler
 * @irq: Linux interrupt number
 * @data: Pointer to struct aspeed_espi_flash
 *
 * Updates the status flags under the spinlock and wakes the
 * waitqueue to unblock the MTD thread waiting for completion.
 */
 static irqreturn_t aspeed_espi_flash_irq(int irq, void *data)
 {
	 struct aspeed_espi_flash *espi_flash = data;
	 struct device *dev = &espi_flash->mtd.dev;
	 unsigned long flags;

	 dev_err(dev, "aspeed_espi_flash_irq %d\n", irq);

	 spin_lock_irqsave(&espi_flash->spinlock, flags);

	 if (irq == espi_flash->irq_rx_complt)
		 espi_flash->rx_sts |= ESPI_INT_STS_FLASH_RX_CMPLT;
	 else if (irq == espi_flash->irq_tx_complt)
		 espi_flash->tx_sts |= ESPI_INT_STS_FLASH_TX_CMPLT;
	 else if (irq == espi_flash->irq_rx_abt)
		 espi_flash->rx_sts |= ESPI_INT_STS_FLASH_RX_ABT;
	 else if (irq == espi_flash->irq_tx_abt)
		 espi_flash->tx_sts |= ESPI_INT_STS_FLASH_TX_ABT;
	 else if (irq == espi_flash->irq_tx_err)
		 espi_flash->tx_sts |= ESPI_INT_STS_FLASH_TX_ERR;

	 spin_unlock_irqrestore(&espi_flash->spinlock, flags);
	 wake_up_interruptible(&espi_flash->wq);

	 return IRQ_HANDLED;
 }

static int aspeed_espi_request_virq(struct platform_device *pdev,
				    struct aspeed_espi_flash *espi_flash,
				    const char *name,
				    int *irq)
{
	int ret;

	if (!pdev || !espi_flash || !name || !irq)
		return -EINVAL;

	*irq = platform_get_irq_byname(pdev, name);
	if (*irq < 0) {
		dev_err(&pdev->dev, "failed to get interrupt %s: %d\n", name, *irq);
		return *irq;
	}

	ret = devm_request_threaded_irq(&pdev->dev, *irq, NULL,
					aspeed_espi_flash_irq,
					IRQF_ONESHOT,
					pdev->name, espi_flash);
	if (ret) {
		dev_err(&pdev->dev, "failed to request interrupt %d: %d\n", *irq, ret);
		return ret;
	}

	return 0;
}

static int aspeed_espi_mafs_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct aspeed_espi_flash *espi_flash;
	struct aspeed_espi_flash_dma *dma;
	struct device *dev = &pdev->dev;
	struct mtd_info *mtd;
	int ret;
	u32 reg;

	dev_err(dev, "entering...\n");

	/*
	 * The eSPI specification doesn't specify a way to retrieve the remote flash size.
	 * It must be provided in the device-tree.
	 */
	if (of_property_read_u32(dev->of_node, "aspeed,espi-mafs-size", &reg)) {
		dev_err(dev, "failed to find aspeed,espi-mafs-size property\n");
		return -EINVAL;
	}

	espi_flash = devm_kzalloc(dev, sizeof(struct aspeed_espi_flash), GFP_KERNEL);
	if (!espi_flash)
		return -ENOMEM;

	dev_set_drvdata(dev, espi_flash);

	espi_flash->regmap = regmap;

	/* Wait for remote to boot and to make sure ESPI_CH3_CAP_N_CONF is updated.
	 * On Intel Emmitsburg the functionality is emulated by Intel ME firmware.
	 * It can take a while for it to boot, thus wait longer...
	 */
	ret = aspeed_espi_mafs_ready(dev, MAFS_BOOT_TIMEOUT_MS);
	if (ret)
		return ret;

	/* Bus lock */
	mutex_init(&espi_flash->lock);

	init_waitqueue_head(&espi_flash->wq);

	spin_lock_init(&espi_flash->spinlock);

	/*
	 * Init static DMA buffers.
	 *
	 * Since the default payload size is quite small (64 bytes) and for
	 * TX packets the flash address must be part of the DMA buffer dma_map_single()
	 * is not option. For RX transfer dma_map_single() could be used, but the
	 * DMA address needs to be incremented every 64 bytes and written to HW.
	 * For simplicity always use static DMA buffers and memcpy data in and out.
	 */
	dma = &espi_flash->dma;

	dma->tx_virt = dma_alloc_coherent(dev, PAGE_SIZE, &dma->tx_addr, GFP_KERNEL);
	if (!dma->tx_virt) {
		dev_err(dev, "cannot allocate DMA TX buffer\n");
		return -ENOMEM;
	}

	dma->rx_virt = dma_alloc_coherent(dev, PAGE_SIZE, &dma->rx_addr, GFP_KERNEL);
	if (!dma->rx_virt) {
		dev_err(dev, "cannot allocate DMA RX buffer\n");
		ret = -ENOMEM;
		goto err_out_unmap_tx;
	}

	/* Enable DMA transfers */
	regmap_write(regmap, ESPI_FLASH_TX_DMA, dma->tx_addr);
	regmap_write(regmap, ESPI_FLASH_RX_DMA, dma->rx_addr);
	regmap_set_bits(regmap, ESPI_CTRL, ESPI_CTRL_FLASH_TX_DMA_EN |
			ESPI_CTRL_FLASH_RX_DMA_EN);

	mtd = &espi_flash->mtd;
	mtd->dev.parent = dev;
	mtd->size = reg;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->_erase = aspeed_espi_flash_erase;
	mtd->_read = aspeed_espi_flash_read;
	mtd->_write = aspeed_espi_flash_write;
	mtd->type = MTD_NORFLASH;
	mtd->name = DRV_NAME;
	mtd->writesize = 1;
	mtd->owner = THIS_MODULE;
	mtd->priv = espi_flash;

	/* ESPI_CH3_CAP_N_CONF is only valid when channel is ready */
	regmap_read(regmap, ESPI_CH3_CAP_N_CONF, &reg);

	/* Get maximum eSPI packet payload size */
	switch ((reg & ESPI_CH3_CAP_N_CONF_PAYLOAD_MASK) >>
		ESPI_CH3_CAP_N_CONF_PAYLOAD_SHIFT) {
	case ESPI_CH3_CAP_N_CONF_PAYLOAD_SIZE_64:
		espi_flash->max_payload_size = 64;
		break;
	case ESPI_CH3_CAP_N_CONF_PAYLOAD_SIZE_128:
		espi_flash->max_payload_size = 128;
		break;
	case ESPI_CH3_CAP_N_CONF_PAYLOAD_SIZE_256:
		espi_flash->max_payload_size = 256;
		break;
	default:
		dev_err(dev, "Unsupported payload size. ESPI_CH3_CAP_N=%x\n", reg);
		ret = -EINVAL;
		goto err_out_unmap_rx;
	}

	/* DMA buffer must hold maximum payload and 32-bit address */
	BUG_ON((espi_flash->max_payload_size + sizeof(u32)) >= PAGE_SIZE);

	/* Opt out when running in the wrong mode */
	if (!(reg & ESPI_CH3_CAP_N_CONF_MAFS)) {
		dev_info(dev, "MAFS not supported\n");
		ret = -ENODEV;
		goto err_out_unmap_rx;
	}

	/* erase_mask is send as the Length field when eSPI cycle=ERASE */
	espi_flash->erase_mask = (reg & ESPI_CH3_CAP_N_CONF_ERASE_MASK) >>
				 ESPI_CH3_CAP_N_CONF_ERASE_SHIFT;

	switch (espi_flash->erase_mask) {
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_4KB_64KB:
		/* eSPI specification: "011" is not applicable for erase_mask in MAFS.
		 * Default to 4KB instead.
		 */
		espi_flash->erase_mask = 1;
		mtd->erasesize = 0x1000;
		break;
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_4KB:
		mtd->erasesize = 0x1000;
		break;
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_64KB:
		mtd->erasesize = 0x10000;
		break;
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_128KB:
		mtd->erasesize = 0x20000;
		break;
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_256KB:
		mtd->erasesize = 0x40000;
		break;
	default:
		dev_err(dev, "Unsupported erase size. ESPI_CH3_CAP_N=%x\n", reg);
		ret = -EINVAL;
		goto err_out_unmap_rx;
	}

	/* Request virtual interrupts */
	ret = aspeed_espi_request_virq(pdev, espi_flash, "rx_cmplt", &espi_flash->irq_rx_complt);
	if (ret)
		goto err_out_unmap_rx;
	ret = aspeed_espi_request_virq(pdev, espi_flash, "tx_cmplt", &espi_flash->irq_tx_complt);
	if (ret)
		goto err_out_unmap_rx;
	ret = aspeed_espi_request_virq(pdev, espi_flash, "rx_abt", &espi_flash->irq_rx_abt);
	if (ret)
		goto err_out_unmap_rx;
	ret = aspeed_espi_request_virq(pdev, espi_flash, "tx_abt", &espi_flash->irq_tx_abt);
	if (ret)
		goto err_out_unmap_rx;
	ret = aspeed_espi_request_virq(pdev, espi_flash, "tx_err", &espi_flash->irq_tx_err);
	if (ret)
		goto err_out_unmap_rx;

	/* Set Flash Channel Software Ready */
	regmap_set_bits(regmap, ESPI_CTRL, ESPI_CTRL_FLASH_SW_RDY);

	regmap_read(espi_flash->regmap, 0xc, &reg);
	dev_err(dev, "ESPI_INTEN %x\n", reg);

	/* Register MTD device */
	ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		dev_err(dev, "Failed to register mtd device: %d\n", ret);
		goto err_out_unmap_rx;
	}

	return 0;

err_out_unmap_rx:
	dma_free_coherent(dev, PAGE_SIZE, dma->rx_virt, dma->rx_addr);
err_out_unmap_tx:
	dma_free_coherent(dev, PAGE_SIZE, dma->tx_virt, dma->tx_addr);

	return ret;
}

static void aspeed_espi_mafs_remove(struct platform_device *pdev)
{
	struct aspeed_espi_flash *espi_flash = platform_get_drvdata(pdev);
	struct aspeed_espi_flash_dma *dma = &espi_flash->dma;
	struct device *dev = &pdev->dev;

	regmap_clear_bits(espi_flash->regmap, ESPI_CTRL,
			  ESPI_CTRL_FLASH_TX_DMA_EN |
			  ESPI_CTRL_FLASH_RX_DMA_EN |
			  ESPI_CTRL_FLASH_SW_RDY);

	dma_free_coherent(dev, PAGE_SIZE, dma->tx_virt, dma->tx_addr);
	dma_free_coherent(dev, PAGE_SIZE, dma->rx_virt, dma->rx_addr);
}

static struct platform_driver aspeed_espi_mafs_driver = {
	.driver = {
		.name = DRV_NAME,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.remove_new = aspeed_espi_mafs_remove,
	.probe = aspeed_espi_mafs_probe,
};

module_platform_driver(aspeed_espi_mafs_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("Aspeed SoC eSPI MAFS controller driver");
MODULE_LICENSE("GPL v2");
