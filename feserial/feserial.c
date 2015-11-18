#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <asm/io.h>
#include <uapi/linux/serial_reg.h>

/* Add your code here */
struct feserial_dev {
	void __iomem *regs;
};

static unsigned int reg_read(struct feserial_dev *dev, int offset)
{
	unsigned int val;
	void *addr;
	
	addr = 4 * offset + dev->regs;	

	val = readl(addr);
	
	return val;	
} 

static void reg_write(struct feserial_dev *dev, unsigned int val, int offset)
{
	void *addr;

	addr = 4 * offset + dev->regs;
	writel(val, addr);
}

static void write_char(struct feserial_dev *dev, char c)
{
	/* wait for trasmit buffer to become empty */
	while(!(reg_read(dev, UART_LSR) & UART_LSR_THRE))
		cpu_relax();

	reg_write(dev, c, UART_TX); 
}

static void printString(struct feserial_dev *dev, char *string) 
{
	while(*string != '\0') {
	
		write_char(dev, *string);
		string++;
	}
}

static int feserial_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct feserial_dev  *dev;
	unsigned int uartclk, baud_divisor;

	
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	
	if (!dev) {
		dev_err(&pdev->dev, "Unable to allocate memory for feserial device \r\n");
		return -ENOMEM;
	}	

	pr_info("Called feserial_probe\n");


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	
	if (!res) {
		printk(KERN_ERR "Unable to get IO memory from devicetree \r\n");
		return -1;
	}	
	
	printk("Start address: %x \r\n", res->start);	

	dev->regs = devm_ioremap_resource(&pdev->dev, res);

	if(!dev->regs) {
		dev_err(&pdev->dev, "Cannot remap registers \n");
		return -ENOMEM;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	dev_info(&pdev->dev, "Configuring serial controller \n");

	of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				&uartclk);

	baud_divisor = uartclk / 16 / 115200;
	
	reg_write(dev, 0x7, UART_OMAP_MDR1);
	
	reg_write(dev, 0x0, UART_LCR);
	
	reg_write(dev, UART_LCR_DLAB, UART_LCR);

	reg_write(dev, baud_divisor & 0xFF, UART_DLL);

	reg_write(dev, (baud_divisor >> 8) & 0xFF, UART_DLM);

	reg_write(dev, UART_LCR_WLEN8, UART_LCR);

	/* Request SW reset */
	reg_write(dev, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);
	reg_write(dev, 0x00, UART_OMAP_MDR1);

	printString(dev, "Writing this on UART \r\n");
	
	return 0;
}

static int feserial_remove(struct platform_device *pdev)
{
	pr_info("Called feserial_remove\n");
	pm_runtime_disable(&pdev->dev);

        return 0;
}
#if defined(CONFIG_OF)
static const struct of_device_id my_serial_of_match[] = {
	{ .compatible = "free-electrons,serial" },
	{},
};

MODULE_DEVICE_TABLE(of, my_serial_of_match);
#endif

static struct platform_driver feserial_driver = {
        .driver = {
                .name = "feserial",
                .owner = THIS_MODULE,
		.of_match_table = of_match_ptr(my_serial_of_match),
        },
        .probe = feserial_probe,
        .remove = feserial_remove,
};

module_platform_driver(feserial_driver);
MODULE_LICENSE("GPL");
