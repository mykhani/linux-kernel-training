#define DEBUG
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
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/sched.h>

/* Add your code here */

/* define ioctl numbers */
#define IOCTL_SERIAL_RESET_COUNTER 0
#define IOCTL_SERIAL_GET_COUNTER 1

/* define size for cicular buffer for serial */
#define SERIAL_BUFSIZE 16

struct serial_dev {
	struct miscdevice miscdev;
	struct device *dev;
	void __iomem *regs;
	int irq;
	int characters;
	char serial_buf[SERIAL_BUFSIZE];
	int serial_buf_rd;
	int serial_buf_wr;
	wait_queue_head_t wait; /* workqueue for waiting on serial data */
};

static unsigned int reg_read(struct serial_dev *dev, int offset)
{
	unsigned int val;
	void *addr;
	
	addr = 4 * offset + dev->regs;	

	val = readl(addr);
	
	return val;	
} 

static void reg_write(struct serial_dev *dev, unsigned int val, int offset)
{
	void *addr;

	addr = 4 * offset + dev->regs;
	writel(val, addr);
}

static void write_char(struct serial_dev *dev, char c)
{
	/* wait for trasmit buffer to become empty */
	while(!(reg_read(dev, UART_LSR) & UART_LSR_THRE))
		cpu_relax();

	reg_write(dev, c, UART_TX); 
}

static void printString(struct serial_dev *dev, char *string) 
{
	while(*string != '\0') {
		pr_debug("Writing character %c to UART %x \n", *string, dev->regs);
		write_char(dev, *string);
		string++;
	}
}

static ssize_t serial_write(struct file *filp, const char __user *buf,
                          size_t count, loff_t *f_pos)
{
        struct serial_dev *priv;
	char *data;
	
	priv = container_of(filp->private_data, struct serial_dev, miscdev);

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
static ssize_t serial_read(struct file *filp, char __user *buf, size_t count,
                         loff_t *f_pos)
{
	char *buffer;
	char byte;
	int ret;
	struct serial_dev *priv = container_of(filp->private_data,
						 struct serial_dev, miscdev);

	/* if data is not yet available, wait */
	if (priv->serial_buf_rd == priv->serial_buf_wr) {
		wait_event_interruptible(priv->wait, priv->serial_buf_rd != priv->serial_buf_wr);
	}
	
	buffer = priv->serial_buf;
	
	byte = buffer[priv->serial_buf_rd++];
	/* wrap-around the read position */
	priv->serial_buf_rd %= SERIAL_BUFSIZE;
	/* pass the read byte to userspace */
	if (copy_to_user(buf, &byte, sizeof(char))) {
        	dev_err(priv->dev, "failed to copy data to userland\n");
                ret = -EFAULT;
        }
	
	return (sizeof(char));
}

static long serial_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
        struct serial_dev *priv;
	int ret = 0;

        priv = container_of(filp->private_data, struct serial_dev, miscdev);

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

static const struct file_operations serial_fops = {
        .write          = serial_write,
        .read           = serial_read,
	.unlocked_ioctl = serial_ioctl,
};

/* Interrupt handler */
irqreturn_t serial_interrupt(int irq, void *device)
{
	struct serial_dev *dev = device;
	char *buffer = dev->serial_buf;
	char byte;
	
	byte = reg_read(dev,UART_RX);
	
	pr_debug("Received a character %c \n", byte);
	
	/* write the byte to circular buffer and increment write position */
	buffer[dev->serial_buf_wr++] = byte;
	/* Check for size and roll back */
	dev->serial_buf_wr %= SERIAL_BUFSIZE;

	wake_up(&dev->wait); /* wake up the read thread waiting */
	
	return IRQ_HANDLED;
}
static int serial_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct serial_dev  *dev;
	unsigned int uartclk, baud_divisor;
	unsigned int reg; /* to store the register value */
	int ret;
	
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	
	if (!dev) {
		dev_err(&pdev->dev, "Unable to allocate memory for serial device \r\n");
		return -ENOMEM;
	}	

	pr_info("Called serial_probe\n");


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
	
	dev->irq = platform_get_irq(pdev, 0);
	
	/* initialize waitqueue */
	init_waitqueue_head(&dev->wait);

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

	/* Enable interrupt handler */
	/* Read the contents of UART_IER register */
	reg = reg_read(dev, UART_IER);
	/* Set the bit for enabling receiver interrupt */
	reg |= UART_IER_RDI;
	/* Write back the value to IER register */
	reg_write(dev, reg, UART_IER);

	/* Request SW reset */
	reg_write(dev, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);
	reg_write(dev, 0x00, UART_OMAP_MDR1);

	/* Initialize misc device */
	dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dev->miscdev.name = kasprintf(GFP_KERNEL, "serial-%x", res->start);

        dev->miscdev.fops = &serial_fops;
	/* Register interrupt handler */
	dev_dbg(&pdev->dev, "Registering interrupt handler for IRQ %d \n", dev->irq);

	ret = devm_request_irq(&pdev->dev, dev->irq, serial_interrupt, 0, dev->miscdev.name, dev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register interrupt handler \n");
		goto fail;
	}
	/* store device structure to use dev_err calls */
	dev->dev = &pdev->dev; 

	/* store driver private data */
	platform_set_drvdata(pdev, dev);

	/* register mis device */
	ret = misc_register(&dev->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register misc device \n");
		goto unregister_interrupt;
	}

	//printString(dev, "Writing this on UART \r\n");
	
	return 0;

unregister_interrupt:
	devm_free_irq(dev->dev, dev->irq, dev);
	
fail:
	kfree(dev->miscdev.name);
	return ret; 
}

static int serial_remove(struct platform_device *pdev)
{
	struct serial_dev *dev = platform_get_drvdata(pdev);
	
	if (dev->miscdev.name)
		kfree(dev->miscdev.name);
	
	devm_free_irq(dev->dev, dev->irq, dev);
	misc_deregister(&dev->miscdev);
 
	pr_info("Called serial_remove\n");
	pm_runtime_disable(&pdev->dev);
	

        return 0;
}
#if defined(CONFIG_OF)
static const struct of_device_id my_serial_of_match[] = {
	{ .compatible = "mgc,linux-lab" },
	{},
};

MODULE_DEVICE_TABLE(of, my_serial_of_match);
#endif

static struct platform_driver serial_driver = {
        .driver = {
                .name = "serial",
                .owner = THIS_MODULE,
		.of_match_table = of_match_ptr(my_serial_of_match),
        },
        .probe = serial_probe,
        .remove = serial_remove,
};

module_platform_driver(serial_driver);
MODULE_LICENSE("GPL");
