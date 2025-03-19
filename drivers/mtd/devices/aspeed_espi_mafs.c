// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 ASPEED Technology Inc.
 */
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/mtd/mtd.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>

#include <linux/mfd/aspeed-espi.h>

#define DRV_NAME  "aspeed-espi-mafs"
#define ESPI_FLASH_TIMEOUT 100

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

 #define ESPI_PLD_LEN_MIN	(1UL << 6)
 #define ESPI_PLD_LEN_MAX	(1UL << 12)

struct aspeed_espi_flash_dma {
	void *tx_virt;
	dma_addr_t tx_addr;
	void *rx_virt;
	dma_addr_t rx_addr;
};

/**
 * struct aspeed_espi_flash - driver data
 * @lock:           Prevents concurrent access to the HW
 * @regmap:         Device's regmap. Only direct access registers.
 * @erase_mask:     bitmask encoding erase types that can erase the entire
 *                  flash memory
 * @tx_sts:         TX status set on IRQ
 * @rx_sts:         RX status set on IRQ
 * @wq:             Wait queue for thread work
 * @spinlock:       Spinlock protects rx_sts and tx_sts
 */
struct aspeed_espi_flash {
	struct mutex			lock;
	struct aspeed_espi_flash_dma	dma;
	struct mtd_info			mtd;
	struct regmap			*regmap;
	uint8_t				erase_mask;
	uint32_t			tx_sts;
	uint32_t			rx_sts;
	wait_queue_head_t		wq;
	spinlock_t			spinlock;
};

/**
 * aspeed_espi_ready - Returns true when eSPI MAFS can be used
 * Since there's no interrupt when the eSPI HW is put into reset
 * status BITs must be checked if the MAFS channel is ready for use.
 *
 * @dev: MFD device to check
 *
 */
static bool aspeed_espi_ready(struct device *dev)
{
	struct aspeed_espi_flash *espi_flash = dev_get_drvdata(dev);
	u32 reg;

	/* Test if flash channel is ready */
	regmap_read(espi_flash->regmap, ESPI_STS, &reg);

	if (!(reg & ESPI_CTRL_ENABLED)) {
		dev_info(dev, "eSPI controller not ready\n");
		return false;
	}
	if (!(reg & ESPI_CTRL_FLASH_CHAN_ENABLE)) {
		dev_info(dev, "eSPI flash channel not enabled\n");
		return false;
	}

	regmap_read(espi_flash->regmap, ESPI_CTRL, &reg);

	if (!(reg & ESPI_CTRL_FLASH_CHAN_RDY)) {
		dev_info(dev, "eSPI flash channel not ready\n");
		return false;
	}
	if (!(reg & ESPI_CTRL_RESET_LVL)) {
		dev_info(dev, "eSPI controller is in reset\n");
		return false;
	}

	return true;
}

static long aspeed_espi_flash_rx(struct device *dev, uint8_t *cyc, uint16_t *len,
				 uint32_t *addr, uint8_t *pkt, size_t *pkt_len)
{
	struct aspeed_espi_flash *espi_flash = dev_get_drvdata(dev);
	uint32_t reg, rx_pkt_len, addr_be;
	unsigned long flags;
	void *rx_virt_ptr;
	ulong to;
	int ret;

	/* Test if flash channel is ready */
	if (!aspeed_espi_ready(dev))
		return -EIO;

	spin_lock_irqsave(&espi_flash->spinlock, flags);

	to = msecs_to_jiffies(ESPI_FLASH_TIMEOUT);
	ret = wait_event_interruptible_lock_irq_timeout(espi_flash->wq,
						        espi_flash->rx_sts,
						        espi_flash->spinlock,
						        to);

	spin_unlock_irqrestore(&espi_flash->spinlock, flags);

	if (ret == -ERESTARTSYS)
		return -EINTR;
	else if (!ret)
		return -ETIMEDOUT;
	else if (espi_flash->rx_sts & ESPI_INT_STS_FLASH_RX_ABT)
		return -EFAULT;
	else
		ret = 0;

	/* common header (i.e. cycle type, tag, and length) is taken by HW */
	regmap_read(espi_flash->regmap, ESPI_FLASH_RX_CTRL, &reg);
	*cyc = (reg & ESPI_FLASH_RX_CTRL_CYC_MASK) >> ESPI_FLASH_RX_CTRL_CYC_SHIFT;
	*len = (reg & ESPI_FLASH_RX_CTRL_LEN_MASK) >> ESPI_FLASH_RX_CTRL_LEN_SHIFT;

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	switch (*cyc) {
	case ESPI_FLASH_READ:
	case ESPI_FLASH_WRITE:
	case ESPI_FLASH_ERASE:
		/* FIXME: is this every received in MAFS? */
		/* R/W/E as an address field */
		rx_virt_ptr = espi_flash->dma.rx_virt + sizeof(addr_be);
		rx_pkt_len = (*len) ? *len : ESPI_PLD_LEN_MAX;

		if (addr) {
			memcpy(&addr_be, espi_flash->dma.rx_virt, sizeof(addr_be));

			*addr = ntohl(addr_be);
		}

		break;
	case ESPI_FLASH_SUC_CMPLT_D_MIDDLE:
	case ESPI_FLASH_SUC_CMPLT_D_FIRST:
	case ESPI_FLASH_SUC_CMPLT_D_LAST:
	case ESPI_FLASH_SUC_CMPLT_D_ONLY:
		rx_pkt_len = (*len) ? *len : ESPI_PLD_LEN_MAX;
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
	if (pkt_len)
		*pkt_len = rx_pkt_len;

	if (rx_pkt_len && pkt_len && pkt) {
		if (*pkt_len >= rx_pkt_len)
			memcpy(pkt, rx_virt_ptr, rx_pkt_len);
		else
			ret = -EINVAL;
	}

	spin_lock_irqsave(&espi_flash->spinlock, flags);

	/* Signal that packet has been serviced */
	regmap_write_bits(espi_flash->regmap, ESPI_FLASH_RX_CTRL,
			  ESPI_FLASH_RX_CTRL_PEND_SERV,
			  ESPI_FLASH_RX_CTRL_PEND_SERV);

	espi_flash->rx_sts = 0;

	spin_unlock_irqrestore(&espi_flash->spinlock, flags);

	return ret;
}

static long aspeed_espi_flash_rx_get_completion(struct device *dev)
{
	uint16_t len;
	uint8_t cyc;
	int ret;

	/* Test if flash channel is ready */
	if (!aspeed_espi_ready(dev))
		return -EIO;

	ret = aspeed_espi_flash_rx(dev, &cyc, &len, NULL, NULL, 0);
	if (ret)
		return ret;

	if (cyc != ESPI_FLASH_SUC_CMPLT) {
		dev_info(dev, "Rx response was not successful\n");
		return -EFAULT;
	}
	return 0;
}

static long aspeed_espi_flash_put_tx(struct device *dev, uint8_t cyc, uint16_t len,
				     const uint32_t addr, const uint8_t *pkt, size_t pkt_len)
{
	struct aspeed_espi_flash *espi_flash = dev_get_drvdata(dev);
	unsigned long flags;
	uint32_t reg, addr_be;
	ulong to;
	int ret;


	/* Test if flash channel is ready */
	if (!aspeed_espi_ready(dev))
		return -EIO;

	/* Test if TX ready */
	regmap_read(espi_flash->regmap, ESPI_FLASH_TX_CTRL, &reg);
	if (reg & ESPI_FLASH_TX_CTRL_TRIGGER) {
		dev_err(dev, "Tx operation still pending\n");
		return -EBUSY;
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	addr_be = htonl(addr);
	memcpy(espi_flash->dma.tx_virt, &addr_be, sizeof(addr_be));
	if (pkt && pkt_len) {
		memcpy(espi_flash->dma.tx_virt + sizeof(addr_be), pkt, pkt_len);
	}
	dma_wmb();

	spin_lock_irqsave(&espi_flash->spinlock, flags);
	espi_flash->tx_sts = 0;
	espi_flash->rx_sts = 0;

	reg = ((cyc << ESPI_FLASH_TX_CTRL_CYC_SHIFT) & ESPI_FLASH_TX_CTRL_CYC_MASK)
		| ((len << ESPI_FLASH_TX_CTRL_LEN_SHIFT) & ESPI_FLASH_TX_CTRL_LEN_MASK)
		| ESPI_FLASH_TX_CTRL_TRIGGER;

	/* Signal that packet is ready for TX */
	regmap_write(espi_flash->regmap, ESPI_FLASH_TX_CTRL, reg);

	to = msecs_to_jiffies(ESPI_FLASH_TIMEOUT);
	ret = wait_event_interruptible_lock_irq_timeout(espi_flash->wq,
						        espi_flash->tx_sts,
						        espi_flash->spinlock,
						        to);
	spin_unlock_irqrestore(&espi_flash->spinlock, flags);

	if (ret == -ERESTARTSYS)
		ret = -EINTR;
	else if (!ret) {
		dev_info(dev, "Tx operation not processed within 100msec\n");
		ret = -ETIMEDOUT;
	}
	else if (espi_flash->tx_sts & (ESPI_INT_STS_FLASH_TX_ERR | ESPI_INT_STS_FLASH_TX_ABT)) {
		dev_info(dev, "Tx operation declined by remote\n");
		ret = -EFAULT;
	}
	else
		ret = 0;


	return ret;
}

static irqreturn_t aspeed_espi_flash_irq(int irq, void *data)
{
	struct aspeed_espi_flash *espi_flash = data;
	unsigned long flags;

	if (!(BIT(irq) & ESPI_INT_STS_FLASH_BITS))
		return IRQ_NONE;

	spin_lock_irqsave(&espi_flash->spinlock, flags);
	espi_flash->rx_sts |= BIT(irq) & (ESPI_INT_STS_FLASH_RX_ABT | ESPI_INT_STS_FLASH_RX_CMPLT);
	espi_flash->tx_sts |= BIT(irq) & (ESPI_INT_STS_FLASH_TX_ERR | ESPI_INT_STS_FLASH_TX_ABT | ESPI_INT_STS_FLASH_TX_CMPLT);

	spin_unlock_irqrestore(&espi_flash->spinlock, flags);
	wake_up_interruptible(&espi_flash->wq);

	return IRQ_HANDLED;
}

static int aspeed_espi_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	struct device *dev = &mtd->dev;
	int ret = 0;

	/* Sanity checks */
	if ((uint32_t)instr->len % espi_flash->mtd.erasesize)
		return -EINVAL;

	if ((uint32_t)instr->addr % espi_flash->mtd.erasesize)
		return -EINVAL;

	mutex_lock(&espi_flash->lock);

	while (instr->len) {
		ret = aspeed_espi_flash_put_tx(dev, ESPI_FLASH_ERASE, espi_flash->erase_mask, instr->addr, NULL, 0);
		if (ret)
			goto unlock_mtx_n_out;

		ret = aspeed_espi_flash_rx_get_completion(dev);
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
		len_pt = (len > ESPI_PLD_LEN_MIN) ? ESPI_PLD_LEN_MIN : len;

		ret = aspeed_espi_flash_put_tx(dev, ESPI_FLASH_READ, len_pt, from, NULL, 0);
		if (ret)
			goto unlock_mtx_n_out;

		pkt_len = len_pt;
		len_rx = 0;
		cyc = 0;
		ret = aspeed_espi_flash_rx(dev, &cyc, &len_rx, NULL, buf, &pkt_len);
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

		len -= len_pt;
		buf += len_pt;
		from += len_pt;
		*retlen += len_pt;
	}

unlock_mtx_n_out:
	mutex_unlock(&espi_flash->lock);
	return ret;
}

static int aspeed_espi_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
				   size_t * retlen, const u_char * buf)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	struct device *dev = &mtd->dev;
	uint16_t len_pt;
	int ret = 0;

	mutex_lock(&espi_flash->lock);

	while (len) {
		len_pt = (len > ESPI_PLD_LEN_MIN) ? ESPI_PLD_LEN_MIN : len;

		ret = aspeed_espi_flash_put_tx(dev, ESPI_FLASH_WRITE, len_pt, to, buf, len_pt);
		if (ret)
			goto unlock_mtx_n_out;

		ret = aspeed_espi_flash_rx_get_completion(dev);
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

static int aspeed_espi_mafs_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct aspeed_espi_ctrl *ctrl = dev_get_drvdata(pdev->dev.parent);
	struct aspeed_espi_flash *espi_flash;
	struct aspeed_espi_flash_dma *dma;
	struct device *dev = &pdev->dev;
	struct mtd_info *mtd;
	int ret, irq;
	u32 reg;

	espi_flash = devm_kzalloc(dev, sizeof(struct aspeed_espi_flash), GFP_KERNEL);
	if (!espi_flash)
		return -ENOMEM;

	dev_set_drvdata(dev, espi_flash);
	espi_flash->regmap = regmap;

	/* Bus lock */
	mutex_init(&espi_flash->lock);

	init_waitqueue_head(&espi_flash->wq);

	spin_lock_init(&espi_flash->spinlock);

	dma = &espi_flash->dma;

	dma->tx_virt = dma_alloc_coherent(dev, PAGE_SIZE,
					  &dma->tx_addr, GFP_KERNEL);
	if (!dma->tx_virt) {
		dev_err(dev, "cannot allocate DMA TX buffer\n");
		return -ENOMEM;
	}

	dma->rx_virt = dma_alloc_coherent(dev, PAGE_SIZE,
					  &dma->rx_addr, GFP_KERNEL);
	if (!dma->rx_virt) {
		dev_err(dev, "cannot allocate DMA RX buffer\n");
		ret = -ENOMEM;
		goto err_out_unmap_tx;
	}

	/* Enable DMA transfers */
	regmap_write(regmap, ESPI_FLASH_TX_DMA, dma->tx_addr);
	regmap_write(regmap, ESPI_FLASH_RX_DMA, dma->rx_addr);
	regmap_update_bits(regmap, ESPI_CTRL,
			   ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN,
			   ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN);

	reg = 0;
	/*
	 * The eSPI specification doesn't specify a way to retrieve the remote flash size.
	 * It must be provided in the device-tree.
	 */
	of_property_read_u32(dev->of_node, "aspeed,espi-mafs-size", &reg);

	mtd = &espi_flash->mtd;
	mtd->dev.parent = dev;
	mtd->size = reg;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->_erase = aspeed_espi_flash_erase;
	mtd->_read = aspeed_espi_flash_read;
	mtd->_write = aspeed_espi_flash_write;
	mtd->type = MTD_NORFLASH;
	mtd->name = DRV_NAME;

	regmap_read(regmap, ESPI_CH3_CAP_N_CONF, &reg);
	reg = (reg & ESPI_CH3_CAP_N_CONF_ERASE_MASK) >>
	      ESPI_CH3_CAP_N_CONF_ERASE_SHIFT;
	espi_flash->erase_mask = reg;

	switch (reg) {
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_4KB:
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_4KB_64KB:
		mtd->erasesize = 0x1000;
		espi_flash->erase_mask = 1;
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
		dev_err(dev, "Unsupported erase size %x\n", reg);
		ret = -EINVAL;
		goto err_out_unmap_rx;
	}

	mtd->writesize = 1;
	mtd->owner = THIS_MODULE;
	mtd->priv = espi_flash;

	/* Request virtual interrupts */
	for (int pirq = 0; pirq < 32; pirq++) {
		if (!(BIT(pirq) & ESPI_INT_STS_FLASH_BITS))
			continue;

		irq = regmap_irq_get_virq(ctrl->irq_data, pirq);
		if (irq < 0) {
			dev_err(dev, "failed to get virtual interrupt=%d\n", pirq);
			ret = irq;
			goto err_out_unmap_rx;
		}

		ret = devm_request_threaded_irq(dev, irq, NULL,
						aspeed_espi_flash_irq,
						IRQF_ONESHOT,
						pdev->name, espi_flash);
		if (ret) {
			dev_err(dev, "failed to request interrupt=%d\n", irq);
			goto err_out_unmap_rx;
		}
	}

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
