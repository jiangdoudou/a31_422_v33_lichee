/*  Date: 2011/4/8 11:00:00
 *  Revision: 2.5
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file MCUIF.c
   brief This file contains all function implementations for the MCUIF in linux

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>

#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>
#include <mach/gpio.h>
#include <linux/gpio.h>

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mutex.h>
#include <linux/nsc_gpio.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>

#define MCUIF_DEBUG

#ifdef MCUIF_DEBUG
#define mcu_dbg(x...)	printk(x)
#else
#define mcu_dbg(x...)
#endif

#define MCU_NAME "mcuinf"
#define MCU_FW_V1 0xA1
#define MCU_FW_V2 0xA2
#define MCU_FW_V3 0xA3

//Define MCU REG
#define MCU_REG_MCU_RST		0x10
#define MCU_REG_PW_ON_H		0x11
#define MCU_REG_PW_ON_M		0x12
#define MCU_REG_PW_OFF_H	0x21
#define MCU_REG_PW_OFF_M	0x22
#define MCU_REG_PW_ONOFF_CT	0x31
#define MCU_REG_WATCHDOG_CT	0x32
#define MCU_REG_WATCHDOG_FEED	0x33
#define MCU_REG_DEV_ID1		0x34
#define MCU_REG_DEV_ID2		0x35
#define MCU_REG_DEV_ID3		0x36
#define MCU_REG_DEV_ID4		0x37
#define MCU_REG_DEV_ID5		0x38
#define MCU_REG_DEV_ID6		0x39
#define MCU_REG_FW_VER1		0x3A
#define MCU_REG_FW_VER2		0x3B
#define MCU_REG_CPU_STATUS	0x51

//Define IOCTL
#define	IOCTL_START_POWER_ONOFF 	0xA1
#define	IOCTL_WATCHDOG_ONOFF		0xA2
#define	IOCTL_WATCHDOG_FEED		0xA3	
#define	IOCTL_GET_MCU_FW_VER		0xA4	
#define	IOCTL_GET_DEV_ID		0xA5	
#define	IOCTL_GET_REG_VAL		0xA6	
#define	IOCTL_SET_REG_VAL		0xA7	

#define CPU_PERFORMANCE     1
#define CPU_RECOVER         2

/* Addresses to scan */
static union{
        unsigned short dirty_addr_buf[2];
        const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};

struct mcu_info {
	unsigned char set_value[32];
	unsigned char get_value[32];
};

static __u32 twi_id = 1;
static uint8_t i2c_reg = 0,i2c_data = 0;
static uint8_t g_reg = 0,g_data = 0;
static int gpio_led_hdle = 0;
static struct timer_list timer_task;

struct mcuinterface_data {
        struct i2c_client *mcuinterface_client;
	struct input_dev *input;
};

static char write_cmd(char add,char reg);
static char read_cmd(char add,unsigned char *ReadBackRegValue);
static void get_mcu_fw_version();
//extern int smdt_set_cpu_to_performance(int mode);
//extern int smdt_set_cpu_min_freq(int value);

static uint8_t mcu_fw_version=0x00;
struct mcuinterface_data *mcu_data;
static int major;               /* default to dynamic major */
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

#define DEVNAME "SmdtMcuCom"

/**
 * mcu_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int mcu_fetch_sysconfig_para(void)
{
	int ret = -1;
/*	int device_used = -1;
	__u32 twi_addr = 0;
	char name[I2C_NAME_SIZE];
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
		
	printk("========%s===================\n", __func__);
	 
	if(SCRIPT_PARSER_OK != (ret = script_parser_fetch("mcu_para", "mcu_used", &device_used, 1))){
	                pr_err("%s: script_parser_fetch err.ret = %d. \n", __func__, ret);
	                goto script_parser_fetch_err;
	}
	if(1 == device_used){
		if(SCRIPT_PARSER_OK != script_parser_fetch_ex("mcu_para", "mcu_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
			pr_err("%s: line: %d script_parser_fetch err. \n", __func__, __LINE__);
			goto script_parser_fetch_err;
		}
		if(strcmp(MCU_NAME, name)){
			pr_err("%s: name %s does not match MCU_NAME. \n", __func__, name);
			pr_err(MCU_NAME);
			//ret = 1;
			return ret;
		}
		if(SCRIPT_PARSER_OK != script_parser_fetch("mcu_para", "mcu_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			goto script_parser_fetch_err;
		}
		u_i2c_addr.dirty_addr_buf[0] = twi_addr;
		u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
		printk("%s: after: mcu_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", \
			__func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);

		if(SCRIPT_PARSER_OK != script_parser_fetch("mcu_para", "mcu_twi_id", &twi_id, 1)){
			pr_err("%s: script_parser_fetch err. \n", name);
			goto script_parser_fetch_err;
		}
		printk("%s: twi_id is %d. \n", __func__, twi_id);
		
		ret = 0;
		
	}else{
		pr_err("%s: mcu_unused. \n",  __func__);
		ret = -1;
	}
*/
		u_i2c_addr.dirty_addr_buf[0] = 0x62;
		u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	return 0;

//script_parser_fetch_err:
//	pr_notice("=========script_parser_fetch_err============\n");
//	return ret;

}

/**
 * mcu_detect - Device detection callback for automatic device creation
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int mcu_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	if(twi_id == adapter->nr){
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, MCU_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, MCU_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
		return 0;
	
}

static int mcu_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

//Function as i2c_master_receive, and return 2 if operation is successful.
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, uint16_t len)
{
        struct i2c_msg msgs[1];
        int ret=-1;
        //·¢ËÍÐ´µØÖ·
        msgs[0].flags = !I2C_M_RD;
        msgs[0].addr = client->addr;
        msgs[0].len = 1;                //data address
        msgs[0].buf = buf;
        //½ÓÊÕÊý¾Ý
        msgs[1].flags = I2C_M_RD;//¶ÁÏûÏ¢
        msgs[1].addr = client->addr;
        msgs[1].len = len-1;
        msgs[1].buf = buf+1;

        ret=i2c_transfer(client->adapter, msgs, 1);
        return ret;
}

//Function as i2c_master_send, and return 1 if operation is successful. 
static int i2c_write_bytes(struct i2c_client *client, uint8_t *data, uint16_t len)
{
        struct i2c_msg msg;
        int ret=-1;
        msg.flags = !I2C_M_RD;//Ð´ÏûÏ¢
        msg.addr = client->addr;
        msg.len = len;
        msg.buf = data;

        ret=i2c_transfer(client->adapter, &msg,1);
        return ret;
}

static ssize_t mcuinterface_read_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", g_data);
}

static ssize_t mcuinterface_read_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
        uint8_t i2c_buf[2], i;
        struct i2c_client *client = to_i2c_client(dev);
	
	sscanf(buf, "%d", &i2c_reg);
	i2c_buf[0]=i2c_reg;
	i2c_buf[1]=i2c_data;
	for (i = 0; i < 10; i++){
		if ( -1 == mcu_smbus_read_byte(client, i2c_buf[0], &i2c_buf[1])){
			msleep(200);
			continue;
		}else{
			break;
		}
	}
	g_data = i2c_buf[1];
//	printk("-- i2c data: 0x%x -> %x --\n", i2c_buf[0],i2c_buf[1]);
        return count;
}

static ssize_t mcuinterface_write_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	printk("r:0x%x d:0x%x \n",i2c_reg,i2c_data);
        return sprintf(buf, "%d,%d \n",i2c_reg,i2c_data);
}

static ssize_t mcuinterface_write_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
        unsigned char i2c_buf[2];
        struct i2c_client *client = to_i2c_client(dev);
	int ret,re_write;
        //error = strict_strtoul(buf, 1, &i2c_reg);
	sscanf(buf, "%d,%d", &i2c_reg,&i2c_data);
	printk("r:0x%x d:0x%x \n",i2c_reg,i2c_data);
	i2c_buf[0]=i2c_reg;
	i2c_buf[1]=i2c_data;
	//ret=i2c_write_bytes(client, i2c_buf, 2);
        //msleep(10);
        for(re_write = 5; re_write > 0; re_write--)
        {
		ret=i2c_write_bytes(client, i2c_buf, 2);
                if(ret == 1)            //Initiall success
                        break;
                else
                        msleep(10);
        }
	printk("-- i2c status: %d --\n",ret);
        return count;
}
static DEVICE_ATTR(read,0666,mcuinterface_read_show, mcuinterface_read_store);
static DEVICE_ATTR(write,0666,mcuinterface_write_show, mcuinterface_write_store);


static struct attribute *mcuinterface_attributes[] = {
	&dev_attr_read.attr,
	&dev_attr_write.attr,
	NULL
};

static struct attribute_group mcuinterface_attribute_group = {
	.attrs = mcuinterface_attributes
};

static int mcuinterface_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	uint8_t check_data = 0x0;
	int i, ret = -1;
	
	mcu_dbg("mcuinterface: probe\n");
    printk("-- %d --\n",__LINE__);
	mcu_fw_version=0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
    printk("-- %d --\n",__LINE__);

        mcu_data = kzalloc(sizeof(struct mcuinterface_data), GFP_KERNEL);
        if (!mcu_data) {
                err = -ENOMEM;
                goto exit;
        }

    printk("-- %d --\n",__LINE__);
	i2c_set_clientdata(client, mcu_data);
	mcu_data->mcuinterface_client = client;

    printk("-- %d --\n",__LINE__);
	err = sysfs_create_group(&client->dev.kobj, &mcuinterface_attribute_group);
	if (err < 0)
	{
		mcu_dbg("mcu: sysfs_create_group err\n");
		goto error_sysfs;
	}
    printk("-- %d --\n",__LINE__);
    //--ok--
    for(i = 0; i < 6; i++)
    {
        ret = mcu_smbus_read_byte(client, 0x3a, &check_data);
        if(check_data == 0x89){
            g_data = check_data;
            printk("%s: cjcheng check MCU success! \n", __func__);
            //			printk("%s : %d ret = %d, check_data = %d success\n", __func__, __LINE__, ret, check_data);
            break;
        } else {
            printk("%s: cjcheng check MCU Failed! \n", __func__);
            //			printk("%s : %d ret = %d, check_data = %d error\n", __func__, __LINE__, ret, check_data);
            /*
            if (i == 5){
                //memcpy(0, "err", 3);	
                while(1){
                    __gpio_set_value(gpio_led_hdle, 0);
                    msleep(300);
                    __gpio_set_value(gpio_led_hdle, 1);

                    printk("%s: cjcheng check MCU error! \n", __func__);
                    msleep(300);
                }
                msleep(10);
                check_data = 0;
            }
            */
        }
    }
/*        
	get_mcu_fw_version();

	if(mcu_fw_version>MCU_FW_V1)
		write_cmd(0x51,0x33);  //write power on flag   
*/
// Only for Test 
 
//	write_cmd(49,0);
//        msleep(10);
//	write_cmd(17,0);
//	write_cmd(18,3);
//	write_cmd(33,0);
//	write_cmd(34,3);
//	write_cmd(49,3);
 //       msleep(10);


	return 0;

error_sysfs:
	kfree(mcu_data);
exit:
	return err;
}


static int mcu_init_platform_resource(void)
{
    int ret = -1;
    script_item_u	val;
    script_item_value_type_e  type;

    type = script_get_item("gpio_para", "gpio_pin_1", &val);
    if(SCIRPT_ITEM_VALUE_TYPE_PIO != type)
        printk(KERN_ERR "LED gpio type err! \n");
    gpio_led_hdle = val.gpio.gpio;
    ret = gpio_request(val.gpio.gpio, NULL);
    if (ret != 0) {
        printk(KERN_ERR "ERROR: LED Gpio_request is failed\n");
    }

    if (0 != sw_gpio_setall_range(&val.gpio, 1)) {
        printk(KERN_ERR "IR gpio set err!");
        return -1;
    }

    return ret;
}

static int mcuinterface_remove(struct i2c_client *client)
{
        struct mcuinterface_data *data = i2c_get_clientdata(client);
        sysfs_remove_group(&client->dev.kobj, &mcuinterface_attribute_group);
        kfree(data);
	return 0;
}


static int MCU_open(struct inode *inode, struct file *file)
{
        printk("-- %s --\n",__FUNCTION__);
        return 0;
}

static int MCU_close(struct inode *inode, struct file *file)
{
        printk("-- %s --\n",__FUNCTION__);
        return 0;
}
/*
static void set_cpufreq_timer(unsigned long data)
{
	printk("--  Set CPU Min Freq  --\n");
	//smdt_set_cpu_to_performance(CPU_PERFORMANCE);
	smdt_set_cpu_min_freq(336000);
}
*/
static void get_mcu_fw_version()
{
	struct i2c_client *client = mcu_data->mcuinterface_client;
	uint8_t tmp;
	int ReTryCounter = 0;
ReGetFWVer: 
	mcu_smbus_read_byte(client,0x3b, &mcu_fw_version);
	tmp=mcu_fw_version;
	msleep(10);
	mcu_smbus_read_byte(client,0x3b, &mcu_fw_version);
	if ((tmp != mcu_fw_version)&&(ReTryCounter<10))
	{
		printk("ReGet MCU FW Version(%d:%d)!",tmp,mcu_fw_version);
		ReTryCounter++; 
		goto ReGetFWVer;
	}
	printk("MCU FW Ver:%d \n", mcu_fw_version);
/*
	if (mcu_fw_version==0xA1)
		mcu_fw_version=MCU_FW_V1;
	else if (mcu_fw_version==0xA2)
		mcu_fw_version=MCU_FW_V2;
	else if (mcu_fw_version==0xA3)
		mcu_fw_version=MCU_FW_V3;
	else 
		mcu_fw_version=0x00;
*/
	 if (mcu_fw_version < 0xA1)
                mcu_fw_version=0xA0;


/*	if (mcu_fw_version<MCU_FW_V3)
	{
		printk("-- set min freq --\n");
	        timer_task.function = set_cpufreq_timer;
                timer_task.expires = jiffies + 10*HZ;
		add_timer(&timer_task);
	}
	*/
}

static char write_cmd(char add,char reg)
{
        unsigned char i2c_buf[32];
	uint8_t ReadBackRegValue=0x00;
	struct i2c_client *client = mcu_data->mcuinterface_client;
	int ret,re_write,i2c_write_len;

	i2c_buf[0]=add;
	i2c_buf[1]=reg;

	i2c_write_len=0;

        for(re_write = 20; re_write > 0; re_write--)
        {
		if(mcu_fw_version>MCU_FW_V1)
		{
			ret=i2c_write_bytes(client, i2c_buf, 2);
			if ( add== MCU_REG_WATCHDOG_FEED)
			{
				printk("i2c ret:%d \n",ret);
				if (ret < 0 )
				    msleep(2);
				else
				{
				    printk("Feed WatchDog Not Read Value!\n");
				    return 2;
				}
			}else
			{
			    msleep(10);
			    mcu_smbus_read_byte(client,(add+0x80), &ReadBackRegValue);
			    if(ReadBackRegValue==reg)            //Initiall success
			    {
				i2c_write_len=2;
                        	break;
			    }
			    else
			    {
				i2c_write_len=0;
                       		msleep(10);
			    }
			}
		}else
		{
			ret=i2c_write_bytes(client, i2c_buf, 2);
			if(ret == 1)            //Initiall success
			{
				i2c_write_len=2;
				break;
			}
			else
			{
			        i2c_write_len=0;
			        msleep(10);
			 }
		}
        }
	printk("add:0x%x value:0x%x read alue:0x%x w_time:0x%x len:0x%x \n",add,reg,ReadBackRegValue,(21-re_write),i2c_write_len);
	return  i2c_write_len;
}

static char read_cmd(char add,unsigned char *ReadBackRegValue)
{
	char ret;
	struct i2c_client *client = mcu_data->mcuinterface_client;
	ret=-1;
	if(mcu_fw_version>MCU_FW_V1)
		ret=mcu_smbus_read_byte(client,add, ReadBackRegValue);
        else
		printk("This FW Version Not Suport Read !\n");
	printk("add:0x%x read alue:0x%x \n",add,*ReadBackRegValue);
	return  ret;
}



static ssize_t MCU_write(struct file *file, const char __user *data,
                                   size_t len, loff_t *ppos)
{
        unsigned char i2c_buf[32];
        char c,RegAdd,RegValue;
	struct i2c_client *client = mcu_data->mcuinterface_client;
	int ret,re_write,i2c_write_len;

        if (get_user(c, data))
	{
		printk("Error : Get user data error!\n");
        	return -EFAULT;
	}
	RegAdd=c;
        if (get_user(c, data+1))
	{
		printk("Error : Get user data error!\n");
        	return -EFAULT;
	}
	RegValue=c;

	printk("Add: %d  Value: %d \n",RegAdd,RegValue);

	i2c_write_len = write_cmd(RegAdd,RegValue);

        return i2c_write_len;
}

int WritePowerOffFlagToMCU(void)
{

	if(mcu_fw_version>MCU_FW_V1)
		write_cmd(0x51,0x55);  //write power off flag   

	return 0;
}
EXPORT_SYMBOL(WritePowerOffFlagToMCU);

static ssize_t MCU_read(struct file *file, char __user *buf, size_t len,
                                  loff_t *ppos)
{
        return 1;
}

static long MCU_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    	unsigned char type,tmp_value;
	struct mcu_info  McuInfo;
	int ret;

        printk("-- %s --\n",__FUNCTION__);
	copy_from_user((char *)&McuInfo, (char *)arg,sizeof(struct mcu_info));
	
    	switch(cmd)
    	{
	    case IOCTL_START_POWER_ONOFF:			
    		mcu_dbg("[## smit ##] cmd[%d] IOCTL_START_POWER_ONOFF\n", cmd);
		ret=write_cmd(MCU_REG_PW_ON_H,McuInfo.set_value[0]);
		if (ret<2)
		{
			printk("Write reg error!\n");
			return -1;
		}
		if ((McuInfo.set_value[1]<3 )&&(McuInfo.set_value[0]==0))
		{
		    printk("-- power on min to low ,set to 3 min--\n");
		    McuInfo.set_value[1]=3;
		}
		ret=write_cmd(MCU_REG_PW_ON_M,McuInfo.set_value[1]);
		if (ret<2)
		{
			printk("Write reg error!\n");
			return -1;
		}
		ret=write_cmd(MCU_REG_PW_OFF_H,McuInfo.set_value[2]);
		if (ret<2)
		{
			printk("Write reg error!\n");
			return -1;
		}
		if ((McuInfo.set_value[3]<3)&&(McuInfo.set_value[2]==0))
		{
		    printk("-- power off min to low ,set to 3 min--\n");
		    McuInfo.set_value[3]=3;
		}
		ret=write_cmd(MCU_REG_PW_OFF_M,McuInfo.set_value[3]);
		if (ret<2)
		{
			printk("Write reg error!\n");
			return -1;
		}
		ret=write_cmd(MCU_REG_PW_ONOFF_CT,McuInfo.set_value[4]);
		if (ret<2)
		{
			printk("Write reg error!\n");
			return -1;
		}
	        break;

	    case IOCTL_WATCHDOG_ONOFF:			
    		mcu_dbg("[## smit ##] cmd[%d] IOCTL_WATCHDOG_ONOFF:\n", cmd);
		ret=write_cmd(MCU_REG_WATCHDOG_CT,McuInfo.set_value[0]);
		if (ret<2)
		{
			printk("Write reg error!\n");
			return -1;
		}
	        break;

	    case IOCTL_WATCHDOG_FEED:			
    		mcu_dbg("[## smit ##] cmd[%d] IOCTL_WATCHDOG_FEED:\n", cmd);
		ret=write_cmd(MCU_REG_WATCHDOG_FEED,0xAB);
		if (ret<2)
		{
			printk("Write reg error!\n");
			return -1;
		}
	        break;

	    case IOCTL_GET_MCU_FW_VER:			
    		mcu_dbg("[## smit ##] cmd[%d] IOCTL_MCU_FW_VER:\n", cmd);
		ret=read_cmd(MCU_REG_FW_VER2,&tmp_value);
		if (ret<0)
		{
			printk("Write reg error!\n");
			return -1;
		}
		McuInfo.get_value[0]=tmp_value;

		copy_to_user((char *)arg,(char *)&McuInfo, sizeof(struct mcu_info));

	        break;

	    case IOCTL_GET_DEV_ID:			
    		mcu_dbg("[## smit ##] cmd[%d] IOCTL_GET_DEV_ID:\n", cmd);
		ret=read_cmd(MCU_REG_DEV_ID1,&tmp_value);
		if (ret<0)
		{
			printk("Write reg error!\n");
			return -1;
		}
		McuInfo.get_value[0]=tmp_value;

		ret=read_cmd(MCU_REG_DEV_ID2,&tmp_value);
		if (ret<0)
		{
			printk("Write reg error!\n");
			return -1;
		}
		McuInfo.get_value[1]=tmp_value;

		ret=read_cmd(MCU_REG_DEV_ID3,&tmp_value);
		if (ret<0)
		{
			printk("Write reg error!\n");
			return -1;
		}
		McuInfo.get_value[2]=tmp_value;


		ret=read_cmd(MCU_REG_DEV_ID4,&tmp_value);
		if (ret<0)
		{
			printk("Write reg error!\n");
			return -1;
		}
		McuInfo.get_value[3]=tmp_value;

		ret=read_cmd(MCU_REG_DEV_ID5,&tmp_value);
		if (ret<0)
		{
			printk("Write reg error!\n");
			return -1;
		}
		McuInfo.get_value[4]=tmp_value;

		ret=read_cmd(MCU_REG_DEV_ID6,&tmp_value);
		if (ret<0)
		{
			printk("Write reg error!\n");
			return -1;
		}
		McuInfo.get_value[5]=tmp_value;
		copy_to_user((char *)arg,(char *)&McuInfo, sizeof(struct mcu_info));
	        break;

	    case IOCTL_GET_REG_VAL:			
    		mcu_dbg("[## smit ##] cmd[%d] IOCTL_GET_REG_VAL:\n", cmd);
		ret=read_cmd(McuInfo.set_value[0],&tmp_value);
		if (ret<0)
		{
			printk("Write reg error!\n");
			return -1;
		}
		McuInfo.get_value[0]=tmp_value;

		copy_to_user((char *)arg,(char *)&McuInfo, sizeof(struct mcu_info));
	        break;

	    case IOCTL_SET_REG_VAL:			
    		mcu_dbg("[## smit ##] cmd[%d] IOCTL_SET_REG_VAL:\n", cmd);
		ret=write_cmd(McuInfo.set_value[0],McuInfo.set_value[1]);
		if (ret<2)
		{
			printk("Write reg error!\n");
			return -1;
		}
	        break;

	    default :
    		mcu_dbg("Eror! [## smit ##] cmd[%d] Not found!\n", cmd);
		return -1;
	}
	return 0;
};

static struct file_operations mcu_fileops = {
	.owner  = THIS_MODULE,
	.open   = MCU_open,
	.release= MCU_close,
	.write  = MCU_write,
	.read   = MCU_read,
	.unlocked_ioctl	= MCU_ioctl,
};

static const struct i2c_device_id mcuinterface_id[] = {
	{ MCU_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mcuinterface_id);

static struct i2c_driver mcuinterface_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= MCU_NAME,
	},
	.id_table	= mcuinterface_id,
	.probe		= mcuinterface_probe,
	.remove		= mcuinterface_remove,
	.address_list	= u_i2c_addr.normal_i2c,
};

static struct cdev mcu_cdev;
static struct class     *mcu_class;

static int __init MCUIF_init(void)
{
	int ret = -1;
	int result;
        int rc;
	dev_t devid;
	mcu_dbg("mcuinterface: init\n");
	if(mcu_fetch_sysconfig_para()){
		printk("%s: err.\n", __func__);
		return -1;
	}

	result = mcu_init_platform_resource();
	if(0 != result){
	    printk("%s: init_platform_resource err. \n", __func__);    
	}

	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	mcuinterface_driver.detect = mcu_detect;
	
        if (major) {
                devid = MKDEV(major, 0);
                rc = register_chrdev_region(devid, 1, DEVNAME);
        } else {
                rc = alloc_chrdev_region(&devid, 0, 1, DEVNAME);
                major = MAJOR(devid);
        }
	

        /* ignore minor errs, and succeed */
        cdev_init(&mcu_cdev, &mcu_fileops);
        cdev_add(&mcu_cdev, devid, 1);

	mcu_class = class_create(THIS_MODULE, "mcu_class");
    	if (IS_ERR(mcu_class))
    	{
        	printk("create class error\n");
        	return -1;
    	}

        device_create(mcu_class, NULL, devid, NULL, "McuCom");
	ret = i2c_add_driver(&mcuinterface_driver);
	init_timer(&timer_task);
	return ret;
}

static void __exit MCUIF_exit(void)
{
	i2c_del_driver(&mcuinterface_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("MCUIF driver");
MODULE_LICENSE("GPL");

module_init(MCUIF_init);
module_exit(MCUIF_exit);

