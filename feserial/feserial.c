#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <asm/io.h>
#include <uapi/linux/serial_reg.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>

/* Add your code here */

/* define ioctl numbers */
#define IOCTL_SERIAL_RESET_COUNTER 0
#define IOCTL_SERIAL_GET_COUNTER 1

struct feserial_dev {
	struct miscdevice miscdev;
	struct device *dev;
	void __iomem *regs;
	int characters;
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

static ssize_t feserial_write(struct file *filp, const char __user *buf,
                          size_t count, loff_t *f_pos)
{
        struct feserial_dev *priv;
	char *data;
	
	priv = container_of(filp->private_data, struct feserial_dev, miscdev);

	dev_info(priv->dev, "Writing to UART address %p \n", priv->regs);
	
	/* allocate memory for data to be written */
	data = kmalloc(sizeof(char) * count, GFP_KERNEL);

	if (!data) {
		dev_err(priv->dev, "Unable to allocate kernel buffer for data \n");
		return -ENOMEM;
	}

        if (copy_from_user(data, buf, count))
                return -EFAULT;
	
	printString(priv, data);

	kfree(data);
	/* Increment the characters count */
	priv->characters += count;

	return count;
	
}
/* patch the kernel to get private data from file pointer */
static ssize_t feserial_read(struct file *filp, char __user *buf, size_t count,
                         loff_t *f_pos)
{
#if 0
        struct fpga_dev *priv = filp->private_data;
        return simple_read_from_buffer(buf, count, f_pos,
                                       priv->vaddr, priv->bytes);
#endif
	return -EINVAL;
}

static long feserial_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
        struct feserial_dev *priv;
	int ret = 0;

        priv = container_of(filp->private_data, struct feserial_dev, miscdev);

        dev_dbg(priv->dev, "IOCTL cmd = 0x%x", cmd);

        switch (cmd) {
        case IOCTL_SERIAL_RESET_COUNTER:
                dev_dbg(priv->dev, ": IOCTL_SERIAL_RESET_COUNTER.\n");
		/* reset character count here */
		priv->characters = 0;
		break;
	case IOCTL_SERIAL_GET_COUNTER:
        	dev_dbg(priv->dev, ": IOCTL_SERIAL_GET_COUNTER.\n");
		if (copy_to_user((int __user *)data, &priv->characters, sizeof(int))) {
			dev_err(priv->dev, "failed to copy data to userlan\n");
			ret = -EFAULT;
		}
		break;
        default:
                dev_err(priv->dev, ": unsupported ioctl %d.\n", cmd);
                ret = -ENOIOCTLCMD;
        }

        return ret;
}


static const struct file_operations feserial_fops = {
        .write          = feserial_write,
        .read           = feserial_read,
	.unlocked_ioctl = feserial_ioctl,
};

static int feserial_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct feserial_dev  *dev;
	unsigned int uartclk, baud_divisor;
	int ret;
	
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

	/* Initialize misc device */
	dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dev->miscdev.name = kasprintf(GFP_KERNEL, "feserial-%x", res->start);

        dev->miscdev.fops = &feserial_fops;

	/* store device structure to use dev_err calls */
	dev->dev = &pdev->dev; 

	/* store driver private data */
	platform_set_drvdata(pdev, dev);

	/* register mis device */
	ret = misc_register(&dev->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register misc device \n");
		goto fail;
	}

	//printString(dev, "Writing this on UART \r\n");
	
	return 0;

fail:
	kfree(dev->miscdev.name);
	return ret; 
}

static int feserial_remove(struct platform_device *pdev)
{
	struct feserial_dev *dev = platform_get_drvdata(pdev);
	
	if (dev->miscdev.name)
		kfree(dev->miscdev.name);
	
	misc_deregister(&dev->miscdev);
 
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
