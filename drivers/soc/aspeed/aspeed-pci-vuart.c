#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/serial_8250.h>
#include <linux/serial_core.h>

#define ASPEED_VENDOR_ID 0x1a03
#define ASPEED_DEVICE_ID 0x2402
#define VUART1_BAR1_OFFSET 0xFE0

static int line = -1;

static int aspeed_pci_vuart_probe(struct pci_dev *pdev,
                                  const struct pci_device_id *id) {
  struct uart_8250_port uart = {};
  resource_size_t bar1_start;
  void __iomem *base;
  int rc;

  rc = pcim_enable_device(pdev);
  if (rc)
    return rc;

  bar1_start = pci_resource_start(pdev, 1);
  if (!bar1_start) {
    dev_err(&pdev->dev, "BAR1 not assigned\n");
    return -ENODEV;
  }

  base = pcim_iomap(pdev, 1, 0);
  if (!base)
    return -ENOMEM;

  uart.port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE;
  uart.port.type = PORT_16550A;
  uart.port.iotype = UPIO_MEM32;
  uart.port.regshift = 2;
  uart.port.uartclk = 115200 * 16; /* base_baud = 115200 */
  uart.port.mapbase = bar1_start + VUART1_BAR1_OFFSET;
  uart.port.membase = base + VUART1_BAR1_OFFSET;
  uart.port.irq = 0; /* polled mode */
  uart.port.dev = &pdev->dev;

  line = serial8250_register_8250_port(&uart);
  if (line < 0) {
    dev_err(&pdev->dev, "register failed: %d\n", line);
    return line;
  }

  dev_info(&pdev->dev, "VUART1 registered as ttyS%d at MMIO 0x%llx\n", line,
           (u64)uart.port.mapbase);
  return 0;
}

static void aspeed_pci_vuart_remove(struct pci_dev *pdev) {
  if (line >= 0)
    serial8250_unregister_port(line);
}

static const struct pci_device_id aspeed_pci_vuart_ids[] = {
    {PCI_DEVICE(ASPEED_VENDOR_ID, ASPEED_DEVICE_ID)},
    {
        0,
    }};
MODULE_DEVICE_TABLE(pci, aspeed_pci_vuart_ids);

static struct pci_driver aspeed_pci_vuart_driver = {
    .name = "aspeed-pci-vuart",
    .id_table = aspeed_pci_vuart_ids,
    .probe = aspeed_pci_vuart_probe,
    .remove = aspeed_pci_vuart_remove,
};
module_pci_driver(aspeed_pci_vuart_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASPEED AST2600 PCIe VUART1 host driver");