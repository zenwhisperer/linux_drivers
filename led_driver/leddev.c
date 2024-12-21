#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/ide.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define DEV_MAJOR 200
#define DEV_NAME "leddev"
#define DEV_VERS "4.1-DTS"

typedef struct device_info 
{
    struct cdev cdev;
    dev_t devid;
    struct class * class;
    struct device * device;
    int major;
    int minor;
    struct device_node * nd;
} device_info_t;
device_info_t devinfo;
// static char readbuf[100];
static char writebuf[100];

static void __iomem * VM_CCM_CCGR1_BASE;
static void __iomem * VM_SW_MUX_GPIO1_IO03_BASE;
static void __iomem * VM_SW_PAD_GPIO1_IO03_BASE;
static void __iomem * VM_GPIO1_GDIR_BASE;
static void __iomem * VM_GPIO1_DR_BASE;

static int dev_open(struct inode *inode, struct file *filp)
{
    printk("char device: %s open\r\n", DEV_NAME);
    return 0;
}

static int dev_release(struct inode *inode, struct file *filp)
{
    printk("char device: %s close\r\n", DEV_NAME);
    return 0;
}

static ssize_t dev_read(struct file *flip, char __user *buf, size_t cnt, loff_t *offt)
{
    int ret = 0;
    return ret;
}

static int led_on(void)
{
    u32 value = ioread32(VM_GPIO1_DR_BASE);
    value &= ~(0 << 3);
    iowrite32(value, VM_GPIO1_DR_BASE);
    return value;
}
static int led_off(void)
{
    u32 value = ioread32(VM_GPIO1_DR_BASE);
    value |= (1 << 3);
    iowrite32(value, VM_GPIO1_DR_BASE);
    return value;
}

static ssize_t dev_write(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    int ret = 0;
    char loc_buf[1];
    ret = copy_from_user(loc_buf, buf, 1);
    if (ret == 0) {
        printk("char device: %s recv data ok\r\n", DEV_NAME);
        printk("%s", writebuf);
        if (loc_buf[0] == "1") {
            led_on();
        } else if (loc_buf[0] == "0") {
            led_off();
        }
    } else {
        printk("char device: %s recv data failed\r\n", DEV_NAME);
    }

    return ret;
}

static const struct file_operations dev_ops = 
{
    .owner = THIS_MODULE,
    .open = dev_open,
    .release = dev_release,
    .read = dev_read,
    .read = dev_write
};


static int __init dev_init(void)
{
    const char * str;
    u32 regdata[10];

    printk("driver version: %s\n", DEV_VERS);

    // process dts
    devinfo.nd = of_find_node_by_path("/alphaled");
    if (devinfo.nd == NULL) {
        return -EINVAL;
    }
    if (of_property_read_string(devinfo.nd, "status", &str) < 0) {
        return -EINVAL;
    } else {
        printk("status: %s\n", str);
    }
    if (of_property_read_string(devinfo.nd, "compatible", &str) < 0) {
        return -EINVAL;
    } else {
        printk("compatible: %s\n", str);
    }
    if (of_property_read_u32_array(devinfo.nd, "reg", regdata, 10) < 0) {
        return -EINVAL;
    } else {
        u32 i;
        printk("regval:\n");
        for (i = 0; i < 10; i++) {
            printk("%#x ", regdata[i]);
        }
        printk("\r\n");
    }

    VM_CCM_CCGR1_BASE = of_iomap(devinfo.nd, 0);
    VM_SW_MUX_GPIO1_IO03_BASE = of_iomap(devinfo.nd, 1);
    VM_SW_PAD_GPIO1_IO03_BASE = of_iomap(devinfo.nd, 2);
    VM_GPIO1_GDIR_BASE = of_iomap(devinfo.nd, 3);
    VM_GPIO1_DR_BASE = of_iomap(devinfo.nd, 4);
    // VM_CCM_CCGR1_BASE = ioremap(regdata[0], regdata[1]);
    // VM_SW_MUX_GPIO1_IO03_BASE = ioremap(regdata[2], regdata[3]);
    // VM_SW_PAD_GPIO1_IO03_BASE = ioremap(regdata[4], regdata[5]);
    // VM_GPIO1_GDIR_BASE = ioremap(regdata[6], regdata[7]);
    // VM_GPIO1_DR_BASE = ioremap(regdata[8], regdata[9]);

    // enable clk 
    u32 val = readl(VM_CCM_CCGR1_BASE);
    val |= 3 << 26;
    writel(val, VM_CCM_CCGR1_BASE);

    // gpio mode
    val = readl(VM_SW_MUX_GPIO1_IO03_BASE);
    val = 5;
    writel(val, VM_SW_MUX_GPIO1_IO03_BASE);
    val = 0x10b0;
    writel(val, VM_SW_PAD_GPIO1_IO03_BASE);

    //gpio direction 
    val = readl(VM_GPIO1_GDIR_BASE);
    val |= 1 << 3; //output
    writel(val, VM_GPIO1_GDIR_BASE); 

    // gpio data init
    val &= ~(1 << 3); //low level
    writel(val, VM_GPIO1_DR_BASE);


    // allocate a device number
    int rc = 0;
    if (devinfo.major){
        devinfo.devid = MKDEV(devinfo.major, 0);
        rc = register_chrdev_region(devinfo.devid, 1, "test");
    } else {
        rc = alloc_chrdev_region(&devinfo.devid, 0, 1, "test");
        devinfo.major = MAJOR(devinfo.devid);
        devinfo.minor = MINOR(devinfo.devid);
    }
    if (rc < 0) {
        printk("char device: %s chrdev_region failed\r\n", DEV_NAME);
	}

    // register the device
    devinfo.cdev.owner = THIS_MODULE;
    cdev_init(&devinfo.cdev, &dev_ops);
    cdev_add(&devinfo.cdev, devinfo.devid, 1);

    // make a device node
    // medv -> udev create a /dev/xxx
    devinfo.class = class_create(THIS_MODULE, DEV_NAME);
    if (IS_ERR(devinfo.class)) {
        return PTR_ERR(devinfo.class);
    }
    devinfo.device = device_create(devinfo.class, NULL, devinfo.devid, NULL, DEV_NAME);
    if (IS_ERR(devinfo.device)) {
        return PTR_ERR(devinfo.device);
    }
    return rc;
}

static void __exit dev_exit(void)
{
    led_off();
    iounmap(VM_CCM_CCGR1_BASE);
    iounmap(VM_SW_MUX_GPIO1_IO03_BASE);
    iounmap(VM_SW_PAD_GPIO1_IO03_BASE);
    iounmap(VM_GPIO1_GDIR_BASE);
    iounmap(VM_GPIO1_DR_BASE);

    unregister_chrdev_region(devinfo.devid, 1);
    cdev_del(&devinfo.cdev);
    device_destroy(devinfo.class, devinfo.devid);
    class_destroy(devinfo.class);
    printk("char device: %s exit\r\n", DEV_NAME);
}

module_init(dev_init);
module_exit(dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("GLX");