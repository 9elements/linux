#include <linux/pci.h>

#define PLANE_SURF_1_A				0x7019c
#define PLANE_CTL_1_A				0x70180

static int vfio_tgl_display_plane_disable(struct pci_dev* pdev,
		const struct pci_device_id *id);
static void skl_primary_plane_disable(void __iomem *regs);
static inline void intel_de_write_fw(void __iomem *uncore, u32 reg, u32 val);

int vfio_tgl_quirks(struct pci_dev *pdev, const struct pci_device_id *id) {
	int ret;

	pci_warn(pdev, "disabling display primary plane initialized by BIOS\n");
	ret = vfio_tgl_display_plane_disable(pdev, id);
	if (ret)
		pci_err(pdev, "failed to disable primary display plane\n");

	return ret;
}

static int vfio_tgl_display_plane_disable(struct pci_dev* pdev,
		const struct pci_device_id *id)
{
	phys_addr_t phys_addr;
	void __iomem *uncore;
	int mmio_size, ret;

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	mmio_size = 2 * 1024 * 1024;
	phys_addr = pci_resource_start(pdev, 0);
	uncore = ioremap(phys_addr, mmio_size);

	skl_primary_plane_disable(uncore);

	iounmap(uncore);

	pci_disable_device(pdev);

	return 0;
}

static void skl_primary_plane_disable(void __iomem *uncore)
{
	//intel_de_write_fw(dev_priv, PLANE_CTL(PIPE_A, PLANE_PRIMARY), 0);
	intel_de_write_fw(uncore, PLANE_CTL_1_A, 0);
	//intel_de_write_fw(dev_priv, PLANE_SURF(PIPE_A, PLANE_PRIMARY), 0);
	intel_de_write_fw(uncore, PLANE_SURF_1_A, 0);
}

static inline void intel_de_write_fw(void __iomem *uncore, u32 reg, u32 val)
{
	writel(val, uncore + reg);
}
