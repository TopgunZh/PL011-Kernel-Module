#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/amba/bus.h>
#include <linux/amba/serial.h>

#define WRITE_BUFFER_LENGTH 32
#define READ_BUFFER_LENGTH 32
#define SERIAL_RESET_COUNTER 0
#define SERIAL_GET_COUNTER 1

struct feserial_device
{
    void __iomem* regs;
    u64 symbol_counter;
    u32 frequency;
    u32 baudrate;
    u8 write_buffer[WRITE_BUFFER_LENGTH];
    u8 read_buffer[READ_BUFFER_LENGTH];
    struct miscdevice miscdev;
    u32 serial_buf_head;
    u32 serial_buf_tail;
    int irq;
    wait_queue_head_t feserial_wait;
};

static unsigned int reg_read(struct feserial_device* dev, int offset)
{
    return readl(dev->regs + offset);
}

static void reg_write(struct feserial_device* dev, int value, int offset)
{
    writel(value, dev->regs + offset);
}

static void write_char(struct feserial_device* dev, char value)
{
    while ((reg_read(dev, UART01x_FR) & UART01x_FR_TXFF) != 0)
    {
        cpu_relax();
    }

    reg_write(dev, value, UART01x_DR);
    dev->symbol_counter++;
    if (value == '\n')
    {
        write_char(dev, '\r');
    }
}

ssize_t serial_read(struct file* file, char __user* buffer, size_t buffer_size,
                    loff_t* offset)
{
    struct feserial_device* dev =
        container_of(file->private_data, struct feserial_device, miscdev);

    int size = 0;

    wait_event_interruptible(dev->feserial_wait,
                             dev->serial_buf_tail != dev->serial_buf_head);

    while (dev->serial_buf_tail != dev->serial_buf_head)
    {
        if (size == buffer_size)
        {
            break;
        }
        put_user(dev->read_buffer[dev->serial_buf_tail++], buffer + size++);

        dev->serial_buf_tail %= READ_BUFFER_LENGTH;
    }
    return size;
}

ssize_t serial_write(struct file* file, const char __user* user_buffer,
                     size_t buffer_size, loff_t* offset)
{
    int i = 0;
    size_t size = 0;
    struct feserial_device* dev =
        container_of(file->private_data, struct feserial_device, miscdev);
    unsigned error_bytes = 0;

    if (buffer_size > WRITE_BUFFER_LENGTH)
    {
        size = WRITE_BUFFER_LENGTH;
    }
    else
    {
        size = buffer_size;
    }

    error_bytes = copy_from_user(dev->write_buffer, user_buffer, size);

    if (error_bytes != 0)
    {
        printk("Couldn't copy %u bytes", error_bytes);
        return -EINVAL;
    }

    for (i = 0; i < size; i++)
    {
        write_char(dev, dev->write_buffer[i]);
    }

    return size;
}

long serial_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    struct feserial_device* dev =
        container_of(file->private_data, struct feserial_device, miscdev);
    switch (cmd)
    {
        case SERIAL_RESET_COUNTER:
            dev->symbol_counter = 0;
            return 0;
        case SERIAL_GET_COUNTER:
            return put_user(dev->symbol_counter, (unsigned int __user*)arg);
        default:
            return -EINVAL;
    }
}

struct file_operations serial_fops = {
    .read = serial_read,
    .write = serial_write,
    .unlocked_ioctl = serial_ioctl,
    .owner = THIS_MODULE,
};

static irqreturn_t feserial_irq(int irq, void* dev_id)
{
    struct feserial_device* local_dev = (struct feserial_device*)dev_id;
    u8 symbol = reg_read(local_dev, UART01x_DR);

    local_dev->read_buffer[local_dev->serial_buf_head++] = symbol;
    local_dev->serial_buf_head %= READ_BUFFER_LENGTH;

    wake_up(&local_dev->feserial_wait);
    return IRQ_HANDLED;
}

static int feserial_probe(struct platform_device* pdev)
{
    u32 baud_rate_divisor = 0;

    struct resource* res;
    struct device_node* clk_node;
    struct feserial_device* local_dev =
        devm_kzalloc(&pdev->dev, sizeof(struct feserial_device), GFP_KERNEL);

    if (!local_dev)
    {
        dev_err(&pdev->dev, "Can't allocate enough space for driver data");
        return -ENOMEM;
    }
    local_dev->baudrate = 115200;
    local_dev->symbol_counter = 0;
    local_dev->serial_buf_head = 0;
    local_dev->serial_buf_tail = 0;
    init_waitqueue_head(&local_dev->feserial_wait);
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    if (!res)
    {
        dev_err(&pdev->dev, "Can't load base address from device tree\n");
        return -EIO;
    }

    /* Enable power management */
    pm_runtime_enable(&pdev->dev);
    pm_runtime_get_sync(&pdev->dev);
    /*                         */

    /* Print base address */
    dev_info(&pdev->dev, "Serial base address: %X", res->start);
    /*                    */

    /* Remap base address */
    local_dev->regs = devm_ioremap_resource(&pdev->dev, res);
    if (!local_dev->regs)
    {
        dev_err(&pdev->dev, "Can't remap registers\n");
        return -ENOMEM;
    }
    /*                    */

    /* Get frequency device_node */
    clk_node = of_parse_phandle(pdev->dev.of_node, "clocks", 0);
    if (!clk_node)
    {
        dev_err(&pdev->dev, "Can't get clk node\n");
        return -EBUSY;
    }
    /*               */
    of_property_read_u32(clk_node, "clock-frequency", &local_dev->frequency);

    /* Turn off UART and set baudrate*/

    // Disable UART
    reg_write(local_dev, reg_read(local_dev, UART011_CR) & (~UART01x_CR_UARTEN),
              UART011_CR);

    /* baud_divisor = local_dev->frequency / 16 / local_dev->baudrate */

    baud_rate_divisor = local_dev->frequency << 6;
    baud_rate_divisor /= (local_dev->baudrate << 4);

    reg_write(local_dev, (baud_rate_divisor >> 6), UART011_IBRD);
    reg_write(local_dev, (0x3F & baud_rate_divisor), UART011_FBRD);

    // Configure Line Options
    reg_write(local_dev, UART01x_LCRH_WLEN_8 | UART01x_LCRH_FEN, UART011_LCRH);
    // Enable Uart and set TX/RX mode.
    reg_write(local_dev,
              reg_read(local_dev, UART011_CR) | UART011_CR_RXE |
                  UART011_CR_TXE | UART01x_CR_UARTEN,
              UART011_CR);

    // FIFO select TX/RX 1/8

    reg_write(local_dev, UART011_IFLS_RX1_8 | UART011_IFLS_TX1_8, UART011_IFLS);

    // Get irq

    reg_write(local_dev, UART011_RXIM, UART011_IMSC);
    local_dev->irq = platform_get_irq(pdev, 0);
    if (local_dev->irq < 0)
    {
        return -ENXIO;
    }
    if (devm_request_irq(&pdev->dev, local_dev->irq, feserial_irq,
                         IRQF_TRIGGER_RISING, pdev->name, local_dev) < 0)
    {
        return -ENXIO;
    }
    /* Register misc device */

    local_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
    local_dev->miscdev.name =
        devm_kasprintf(&pdev->dev, GFP_KERNEL, "feserial-%x", res->start);
    local_dev->miscdev.fops = &serial_fops;

    platform_set_drvdata(pdev, local_dev);

    return misc_register(&local_dev->miscdev);
}

static int feserial_remove(struct platform_device* pdev)
{
    struct feserial_device* local_dev;
    /* Disable power management */
    pm_runtime_disable(&pdev->dev);
    /*                         */

    /* Unregister misc device */
    local_dev = platform_get_drvdata(pdev);
    misc_deregister(&local_dev->miscdev);

    pr_info("Called feserial_remove\n");
    return 0;
}

static struct of_device_id feserial_dt_match[] = {
    {.compatible = "rtrk,serial"},
    {},
};

static struct platform_driver feserial_driver = {
    .driver =
        {
            .name = "feserial",
            .owner = THIS_MODULE,
            .of_match_table = of_match_ptr(feserial_dt_match),
        },
    .probe = feserial_probe,
    .remove = feserial_remove,
};

module_platform_driver(feserial_driver);
MODULE_LICENSE("GPL");
