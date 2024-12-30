#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/ide.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of_address.h>
#include <linux/timer.h>

#define DEV_NAME "leddev"
#define DEV_VERS "5.0-led-timer"

typedef struct device_info 
{
    dev_t devid;
    int major;
    int minor;
    int gpio_num;
    int timer_period;
    struct timer_list timer;
    struct cdev cdev;
    struct class * class;
    struct device * device;
    struct device_node * nd;
} device_info_t;

device_info_t devinfo;

static char writebuf[100];

static int dev_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &devinfo;
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
    gpio_set_value(devinfo.gpio_num, 0);
    return 0;
}
static int led_off(void)
{
    gpio_set_value(devinfo.gpio_num, 1);
    return 1;
}

static ssize_t dev_write(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    int ret = 0;
    char loc_buf[1];
    struct devinfo * dev = filp->private_data;
    ret = copy_from_user(loc_buf, buf, 1);
    printk("recved data:%s\r\n", loc_buf[0]);
    if (ret > 0) {
        printk("char device: %s recv data ok\r\n", DEV_NAME);
        printk("%s\r\n", writebuf);
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

static long dev_unlocked_ioctl(struct file * file, unsigned int cmd, unsigned long arg)
{
    return 0;
}

static const struct file_operations dev_ops = 
{
    .owner = THIS_MODULE,
    .open = dev_open,
    .release = dev_release,
    .read = dev_read,
    .write = dev_write,
    .unlocked_ioctl = dev_unlocked_ioctl,
};

static void timer_cb(unsigned long arg)
{
    // toggle led
    static int flag = 0;
    if (flag == 0) {
        led_off();
        flag = 1;
    } else {
        led_on();
        flag = 0;
    }

    mod_timer(&devinfo.timer, jiffies + msecs_to_jiffies(devinfo.timer_period));

}

static void timer_init(void) 
{
    init_timer(&devinfo.timer);
    devinfo.timer_period = 1000;
    devinfo.timer.function = timer_cb;
    devinfo.timer.data = (unsigned long)&devinfo;
    // the following must be called to start timer
    mod_timer(&devinfo.timer, jiffies + msecs_to_jiffies(devinfo.timer_period));
}

static int __init dev_init(void)
{
    printk("driver version: %s\n", DEV_VERS);

    // allocate a device number
    int rc;
    if (devinfo.major){
        devinfo.devid = MKDEV(devinfo.major, 0);
        rc = register_chrdev_region(devinfo.devid, 1, "heartbeat-led");
    } else {
        rc = alloc_chrdev_region(&devinfo.devid, 0, 1, "heartbeat-led");
        devinfo.major = MAJOR(devinfo.devid);
        devinfo.minor = MINOR(devinfo.devid);
    }
    printk("char device number : major: %d minor: %d \r\n", devinfo.major, devinfo.minor);
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

    // process dts
    devinfo.nd = of_find_node_by_path("/heartbeat-led");
    if (devinfo.nd == NULL) {
        return -EINVAL;
    }
    devinfo.gpio_num = of_get_named_gpio(devinfo.nd, "heartbeat-gpio", 0);
    if (devinfo.gpio_num < 0) {
        printk("can not find heartbeat-gpio\r\n");
        return -EINVAL;
    }
    printk("led gpio num: %d\r\n", devinfo.gpio_num);

    // Before doing this, make sure that the dedicate pin is not occupied
    if (gpio_request(devinfo.gpio_num, "heartbeat-gpio")) {
        printk("can not request heartbeat-gpio\r\n");
        return -EINVAL;
    }
    if (gpio_direction_output(devinfo.gpio_num, 1)) {
        printk("gpio_driection_output failed\r\n");
        return -EINVAL;
    }
    gpio_set_value(devinfo.gpio_num, 0);

    timer_init();

    return 0;
}

static void __exit dev_exit(void)
{
    led_off();

    gpio_free(devinfo.gpio_num);
    cdev_del(&devinfo.cdev);
    device_destroy(devinfo.class, devinfo.devid);
    class_destroy(devinfo.class);
    unregister_chrdev_region(devinfo.devid, 1);
    printk("char device: %s exit\r\n", DEV_NAME);
}

module_init(dev_init);
module_exit(dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("GLX");