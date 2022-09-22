#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>

static unsigned short ignore[]      = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { 0x6F, I2C_CLIENT_END };//设备地址为：01101111（0x6f) 七位 

static unsigned short force_addr[] = {ANY_I2C_BUS,0x6F, I2C_CLIENT_END};
static unsigned short * forces[] = {force_addr, NULL};

static struct i2c_client_address_data addr_data = {
    .normal_i2c	= normal_addr,   //要发出地址信号，并且得到ACK信号，才能确定是否存在这个设备
	.probe		= ignore,
    .ignore		= ignore,
    .forces     = forces, //强制认为存在这个设备
};

static struct i2c_driver SY_driver;

static int major;
static struct class *cls; //自动创建设备节点 
struct i2c_client *SY_client;

static ssize_t SY_read(struct file *filp, char __user *buf, size_t count, loff_t * ppos)
{
	int ret=0;
    static volatile unsigned char  values[1]={0};
    

   values[0]=((char)SY_client->addr);
   
   if ( copy_to_user(buf, (void*)values,count )) { //将地址值buf内存的内容传到用户空间的values 
      ret = -EFAULT;
	  goto out;
	  }
   out:
      return ret;
}


//定义字符设备结构体
static struct file_operations SY_fops = {
    .owner = THIS_MODULE,
	.read  = SY_read,
};

static int SY_detect(struct i2c_adapter *adapter, int address, int kind)
{   
    
    printk("SY_detect\n");
    //构建一个i2c_client结构体；收费数据主要靠它，里面有 .address .adapter .driver
    SY_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	SY_client->addr    = address;
	SY_client->adapter = adapter;
	SY_client->driver  = &SY_driver;
	strcpy(SY_client->name, "SY");
	i2c_attach_client(SY_client);//等要卸载驱动时，会调用 I2C_detach
	printk("SY_probe with name = %s, addr = 0x%x\n", SY_client->name, SY_client->addr);
   
	major = register_chrdev(0, "SY", &SY_fops);//申请字符设备主设备号 

	cls = class_create(THIS_MODULE, "SY");//创建一个类 ，然后在类下面创建一个设备 
	class_device_create(cls, NULL, MKDEV(major, 0), NULL, "SY");

	return 0;
}


static int SY_attach(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, SY_detect);
}

static int SY_detach(struct i2c_client *client)
{
	printk("SY_detach\n");
	class_device_destroy(cls, MKDEV(major, 0));
	class_destroy(cls);
	unregister_chrdev(major, "SY");

	i2c_detach_client(client);//client 结构体 
	kfree(i2c_get_clientdata(client));//释放client的内存 

	return 0;
}

//定义i2c_driver结构体 
static struct i2c_driver SY_driver = {
	.driver = {
		.name	= "SY",
	},
	.attach_adapter = SY_attach,
	.detach_client  = SY_detach,
};

static int SY_init(void)
{
	printk("SY_init\n");
	i2c_add_driver(&SY_driver);//注册i2c驱动 
	return 0;
}

static void SY_exit(void)
{
	printk("SY_exit\n");
	i2c_del_driver(&SY_driver);
}

module_init(SY_init);
module_exit(SY_exit);
MODULE_LICENSE("GPL");














