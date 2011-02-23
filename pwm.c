/*
 Copyright (c) 2010, Scott Ellis
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.
	* Neither the name of the <organization> nor the
	  names of its contributors may be used to endorse or promote products
	  derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY Scott Ellis ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL Scott Ellis BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <linux/init.h> 
#include <linux/module.h>
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>

#include "omap_pwm.h"

/* default frequency of 1 kHz */
//#define DEFAULT_TLDR	0xFFFFFFE0
/* default frequency of 48 Hz */
//#define DEFAULT_TLDR	0xFFFFFD55

/* default 50% duty cycle */
/* TMAR = (0xFFFFFFFF - ((0xFFFFFFFF - (DEFAULT_TLDR + 1)) / 2)) */
//#define DEFAULT_TMAR	0xFFFFFFEF
//#define DEFAULT_TMAR	0xFFFFFEAB

/* default TCLR is off state */
#define DEFAULT_TCLR (GPT_TCLR_PT | GPT_TCLR_TRG_OVFL_MATCH | GPT_TCLR_CE | GPT_TCLR_AR) 

//#define DEFAULT_PWM_FREQUENCY 1024
#define DEFAULT_PWM_FREQUENCY 48

static int defaultFrequency = DEFAULT_PWM_FREQUENCY;
module_param(defaultFrequency, int, S_IRUGO);
MODULE_PARM_DESC(defaultFrequency, "The PWM frequency, power of two, max of 16384");


#define USER_BUFF_SIZE	128

struct gpt {
	u32 timer_num;
	u32 mux_offset;
	u32 gpt_base;
	u32 input_freq;
	u32 old_mux;
	u32 tldr;
	u32 tmar;
	u32 tclr;
	u32 num_freqs;
};

struct pwm_dev {
	int frequency;
	struct semaphore sem;
	struct gpt gpt;
	char *user_buff;
};

static dev_t devt;
static struct cdev cdev;

static struct pwm_dev pwm9_dev;
static struct pwm_dev pwm10_dev;
static struct pwm_dev pwm11_dev;


static int init_mux(struct pwm_dev *pwm_dev)
{
	void __iomem *base;

	base = ioremap(OMAP34XX_PADCONF_START, OMAP34XX_PADCONF_SIZE);
	if (!base) {
		printk(KERN_ALERT "init_mux(): ioremap() failed\n");
		return -1;
	}

	pwm_dev->gpt.old_mux = ioread16(base + pwm_dev->gpt.mux_offset);
	iowrite16(PWM_ENABLE_MUX, base + pwm_dev->gpt.mux_offset);
	iounmap(base);	

	return 0;
}

static int restore_mux(struct pwm_dev *pwm_dev)
{
	void __iomem *base;

	if (pwm_dev->gpt.old_mux) {
		base = ioremap(OMAP34XX_PADCONF_START, OMAP34XX_PADCONF_SIZE);
	
		if (!base) {
			printk(KERN_ALERT "restore_mux(): ioremap() failed\n");
			return -1;
		}

		iowrite16(pwm_dev->gpt.old_mux, base + pwm_dev->gpt.mux_offset);
		iounmap(base);	
	}

	return 0;
}

static int set_pwm_frequency(struct pwm_dev *pwm_dev, int frequency)
{
	void __iomem *base;

	base = ioremap(pwm_dev->gpt.gpt_base, GPT_REGS_PAGE_SIZE);
	if (!base) {
		printk(KERN_ALERT "set_pwm_frequency(): ioremap failed\n");
		return -1;
	}

	if (frequency < 0) {
		frequency = DEFAULT_PWM_FREQUENCY;
	} else {
		/* only powers of two, for simplicity */
		//frequency &= ~0x01;

		if (frequency > (pwm_dev->gpt.input_freq / 2)) 
			frequency = pwm_dev->gpt.input_freq / 2;
		else if (frequency == 0)
			frequency = DEFAULT_PWM_FREQUENCY;
	}

	pwm_dev->frequency = frequency;

	/* PWM_FREQ = 32768 / ((0xFFFF FFFF - TLDR) + 1) */
	pwm_dev->gpt.tldr = 0xFFFFFFFF - ((pwm_dev->gpt.input_freq / pwm_dev->frequency) - 1);

	/* just for convenience */	
	pwm_dev->gpt.num_freqs = 0xFFFFFFFE - pwm_dev->gpt.tldr;	

	iowrite32(pwm_dev->gpt.tldr, base + GPT_TLDR);

	/* initialize TCRR to TLDR, have to start somewhere */
	iowrite32(pwm_dev->gpt.tldr, base + GPT_TCRR);

	iounmap(base);

	return 0;
}

static int pwm_off(struct pwm_dev *pwm_dev)
{
	void __iomem *base;

	base = ioremap(pwm_dev->gpt.gpt_base, GPT_REGS_PAGE_SIZE);
	if (!base) {
		printk(KERN_ALERT "pwm_off(): ioremap failed\n");
		return -1;
	}

	pwm_dev->gpt.tclr &= ~GPT_TCLR_ST;
	iowrite32(pwm_dev->gpt.tclr, base + GPT_TCLR); 
	iounmap(base);

	return 0;
}

static int pwm_on(struct pwm_dev *pwm_dev)
{
	void __iomem *base;

	base = ioremap(pwm_dev->gpt.gpt_base, GPT_REGS_PAGE_SIZE);

	if (!base) {
		printk(KERN_ALERT "pwm_on(): ioremap failed\n");
		return -1;
	}

	/* set the duty cycle */
	iowrite32(pwm_dev->gpt.tmar, base + GPT_TMAR);
	
	/* now turn it on */
	pwm_dev->gpt.tclr = ioread32(base + GPT_TCLR);
	pwm_dev->gpt.tclr |= GPT_TCLR_ST;

	iowrite32(pwm_dev->gpt.tclr, base + GPT_TCLR);
 	iounmap(base);

	return 0;
}

static int scpwm(struct pwm_dev *pwm_dev, int sc)
{
	void __iomem *base;

	base = ioremap(pwm_dev->gpt.gpt_base, GPT_REGS_PAGE_SIZE);
	if (!base) {
		printk(KERN_ALERT "pwm_off(): ioremap failed\n");
		return -1;
	}

	if (sc == 1)
		pwm_dev->gpt.tclr |= GPT_TCLR_SCPWM;
	else
		pwm_dev->gpt.tclr &= ~GPT_TCLR_SCPWM;

	iowrite32(pwm_dev->gpt.tclr, base + GPT_TCLR);
	iounmap(base);

	return 0;
}

static int prescale(struct pwm_dev *pwm_dev, int div)
{
	void __iomem *base;
	int i = 0;
	base = ioremap(pwm_dev->gpt.gpt_base, GPT_REGS_PAGE_SIZE);

	if (!base) {
		printk(KERN_ALERT "pwm_off(): ioremap failed\n");
		return -1;
	}

	while (div > 2) {
		i++;
		div /= 2;
	}

	pwm_dev->gpt.tclr |= GPT_TCLR_PRE; //enable prescaler
	pwm_dev->gpt.tclr &= i << 2; //set prescaler ratio
	iowrite32(pwm_dev->gpt.tclr, base + GPT_TCLR);
	iounmap(base);

	return 0;
}

static int set_duty_cycle(struct pwm_dev *pwm_dev, unsigned int duty_cycle) 
{
	unsigned int new_tmar;

	pwm_off(pwm_dev);

	if (duty_cycle == 0)
		return 0;
 
	new_tmar = (duty_cycle * pwm_dev->gpt.num_freqs) / 100;

	if (new_tmar < 1) 
		new_tmar = 1;
	else if (new_tmar > pwm_dev->gpt.num_freqs)
		new_tmar = pwm_dev->gpt.num_freqs;
		
	pwm_dev->gpt.tmar = pwm_dev->gpt.tldr + new_tmar;
printk("TMAR : 0x%08x\n", pwm_dev->gpt.tmar);	
	return pwm_on(pwm_dev);
}

static ssize_t pwm_read(struct file *filp, char __user *buff, size_t count,
			loff_t *offp)
{
	size_t len;
	unsigned int duty_cycle;
	ssize_t error = 0;

  	struct pwm_dev *pwm_dev = (struct pwm_dev *)filp->private_data;
  
	if (!buff) 
		return -EFAULT;

	/* tell the user there is no more */
	if (*offp > 0) 
		return 0;

	if (down_interruptible(&pwm_dev->sem)) 
		return -ERESTARTSYS;

	if (pwm_dev->gpt.tclr & GPT_TCLR_ST) {
		duty_cycle = (100 * (pwm_dev->gpt.tmar - pwm_dev->gpt.tldr)) 
				/ pwm_dev->gpt.num_freqs;

		snprintf(pwm_dev->user_buff, USER_BUFF_SIZE,
				"PWM%d Frequency %u Hz Duty Cycle %u%%\n",
				pwm_dev->gpt.timer_num, pwm_dev->frequency, duty_cycle);
	}
	else {
		snprintf(pwm_dev->user_buff, USER_BUFF_SIZE,
				"PWM%d Frequency %u Hz Stopped\n",
				pwm_dev->gpt.timer_num, pwm_dev->frequency);
	}

	len = strlen(pwm_dev->user_buff);
 
	if (len + 1 < count) 
		count = len + 1;

	if (copy_to_user(buff, pwm_dev->user_buff, count))  {
		printk(KERN_ALERT "pwm_read(): copy_to_user() failed\n");
		error = -EFAULT;
	}
	else {
		*offp += count;
		error = count;
	}

	up(&pwm_dev->sem);

	return error;	
}

static ssize_t pwm_write(struct file *filp, const char __user *buff, 
			size_t count, loff_t *offp)
{
	size_t len;
	unsigned int duty_cycle;
	ssize_t error = 0;
	
  	struct pwm_dev *pwm_dev = (struct pwm_dev *)filp->private_data;

	if (down_interruptible(&pwm_dev->sem)) 
		return -ERESTARTSYS;

	if (!buff || count < 1) {
		printk(KERN_ALERT "pwm_write(): input check failed\n");
		error = -EFAULT; 
		goto pwm_write_done;
	}
	
	/* we are only expecting a small integer, ignore anything else */
	if (count > 8)
		len = 8;
	else
		len = count;
		
	memset(pwm_dev->user_buff, 0, 16);

	if (copy_from_user(pwm_dev->user_buff, buff, len)) {
		printk(KERN_ALERT "pwm_write(): copy_from_user() failed\n"); 
		error = -EFAULT; 	
		goto pwm_write_done;
	}

	if(pwm_dev->user_buff[0] == '+')	{
		pwm_off(pwm_dev);
		if(pwm_dev->user_buff[1] != '\0')
			pwm_dev->gpt.tmar += simple_strtoul(&pwm_dev->user_buff[1], NULL, 0);
		pwm_on(pwm_dev);
printk("TMAR : 0x%08x\n", pwm_dev->gpt.tmar);
	} else if(pwm_dev->user_buff[0] == '-')	{
		pwm_off(pwm_dev);
		if(pwm_dev->user_buff[1] != '\0')
			pwm_dev->gpt.tmar -= simple_strtoul(&pwm_dev->user_buff[1], NULL, 0);
		pwm_on(pwm_dev);
printk("TMAR : 0x%08x\n", pwm_dev->gpt.tmar);
	} else	{
		duty_cycle = simple_strtoul(pwm_dev->user_buff, NULL, 0);

		set_duty_cycle(pwm_dev, duty_cycle);

	}
	/* pretend we ate it all */
	*offp += count;

	error = count;
pwm_write_done:

	up(&pwm_dev->sem);
	
	return error;
}

static int pwm_open(struct inode *inode, struct file *filp)
{
	int error = 0;

	struct pwm_dev *pwm_dev = 0;
	
	unsigned int minor = iminor(inode);

	switch(minor)	{
		case 9:		pwm_dev = &pwm9_dev;	
			break;
		case 10:	pwm_dev = &pwm10_dev;	
			break;
		case 11:	pwm_dev = &pwm11_dev;
			break;
	}
  	
	if(pwm_dev == 0)
		return -EFAULT;

	filp->private_data = pwm_dev;

	if (down_interruptible(&pwm_dev->sem)) 
		return -ERESTARTSYS;

	if (pwm_dev->gpt.old_mux == 0) {
		if (init_mux(pwm_dev))  
			error = -EIO;
		else if (set_pwm_frequency(pwm_dev, defaultFrequency)) 
			error = -EIO;
	}

	if (!pwm_dev->user_buff) {
		pwm_dev->user_buff = kmalloc(USER_BUFF_SIZE, GFP_KERNEL);
		if (!pwm_dev->user_buff)
			error = -ENOMEM;
	}

	up(&pwm_dev->sem);

	return error;	
}

int pwm_ioctl(struct inode *inode, struct file *filp,
unsigned int cmd, unsigned long arg)
{
	int retval = 0;
  	struct pwm_dev *pwm_dev = (struct pwm_dev *)filp->private_data;

	/*
	* extract the type and number bitfields, and don't decode
	* wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	*/
	if (_IOC_TYPE(cmd) != PWM_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > PWM_IOC_MAXNR)
		return -ENOTTY;

 	switch (cmd) {
		case PWM_ON:
			if (pwm_on(pwm_dev))
				retval = -EIO;
			break;
		case PWM_OFF:
			if (pwm_off(pwm_dev))
				retval = -EIO;
			break;
		case PWM_SET_DUTYCYCLE:
			if (set_duty_cycle(pwm_dev, arg))
				retval = -EIO;
			break;
		case PWM_GET_DUTYCYCLE:
			if (pwm_dev->gpt.tclr & GPT_TCLR_ST) { //PWM is on
				retval = (100 * (pwm_dev->gpt.tmar - pwm_dev->gpt.tldr))
						/ pwm_dev->gpt.num_freqs; //real duty cycle
			} else {
				printk(KERN_ALERT "PWM%d is OFF\n", pwm_dev->gpt.timer_num);
				retval = -EIO;
			}
			break;
		case PWM_SET_FREQUENCY:
			if (set_pwm_frequency(pwm_dev, arg))
				retval = -EIO;
			break;
		case PWM_GET_FREQUENCY:
			retval = pwm_dev->frequency;
			break;
		case PWM_SET_POLARITY:
			if (scpwm(pwm_dev, arg))
				retval = -EIO;
			break;
		case PWM_SET_CLK:
/* Do no change clock source !!! */
/*
			if (pwm_dev->gpt.timer_num == 9) {
				printk(KERN_ALERT "Only 32K clk can be used with GPT9\n");
				retval = -EIO;
			} else {
				if (arg == 1)
					pwm_dev->gpt.input_freq = CLK_13K_FREQ;
				else
					pwm_dev->gpt.input_freq = CLK_32K_FREQ;
			}
*/
		break;
		case PWM_SET_PRE:
			if (prescale(pwm_dev, arg))
				retval = -EIO;
			break;
		case PWM_PLUS_DUTYCYCLE:
			pwm_off(pwm_dev);
			pwm_dev->gpt.tmar++;
			pwm_on(pwm_dev);
			break;
		case PWM_MINUS_DUTYCYCLE:
			pwm_off(pwm_dev);
			if(pwm_dev->gpt.tmar > 0)
				pwm_dev->gpt.tmar--;
			pwm_on(pwm_dev);
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}

	return retval;
}

static struct file_operations pwm_fops = {
	.owner = THIS_MODULE,
	.read = pwm_read,
	.write = pwm_write,
	.open = pwm_open,
	.ioctl = pwm_ioctl,
};

static int __init pwm_init_cdev(void)
{
	int error;

	error = alloc_chrdev_region(&devt, 9, 3, "pwm");	/*	minor number start from 9, count 3*/

	if (error < 0) {
		printk(KERN_ALERT "alloc_chrdev_region() failed: %d \n", 
			error);
		return -1;
	}

	cdev_init(&cdev, &pwm_fops);
	cdev.owner = THIS_MODULE;
	
	error = cdev_add(&cdev, devt, 3);
	if (error) {
		printk(KERN_ALERT "cdev_add() failed: %d\n", error);
		unregister_chrdev_region(devt, 3);
		return -1;
	}	

	return 0;
}
 
static int __init pwm_init(void)
{
	memset(&pwm9_dev, 0, sizeof(struct pwm_dev));

	/* change these 4 values to use a different PWM */
	pwm9_dev.frequency = defaultFrequency;
	pwm9_dev.gpt.timer_num = 9;
	pwm9_dev.gpt.mux_offset = GPT9_MUX_OFFSET;
	pwm9_dev.gpt.gpt_base = PWM9_CTL_BASE;
	pwm9_dev.gpt.input_freq = CLK_SYS_FREQ;	/* GPT9 can only use 13MHz clock source */

	pwm9_dev.gpt.tldr = 0xFFFFFFFF - (pwm9_dev.gpt.input_freq / pwm9_dev.frequency) + 1;
	pwm9_dev.gpt.tmar = 0xFFFFFFFF - ((0xFFFFFFFF - (pwm9_dev.gpt.tldr + 1)) / 2);
	pwm9_dev.gpt.tclr = DEFAULT_TCLR;

	sema_init(&pwm9_dev.sem, 1);

	memset(&pwm10_dev, 0, sizeof(struct pwm_dev));

	/* change these 4 values to use a different PWM */
	pwm10_dev.frequency = defaultFrequency;
	pwm10_dev.gpt.timer_num = 10;
	pwm10_dev.gpt.mux_offset = GPT10_MUX_OFFSET;
	pwm10_dev.gpt.gpt_base = PWM10_CTL_BASE;
	pwm10_dev.gpt.input_freq = CLK_32K_FREQ;

	pwm10_dev.gpt.tldr = 0xFFFFFFFF - (pwm10_dev.gpt.input_freq / pwm10_dev.frequency) + 1;
	pwm10_dev.gpt.tmar = 0xFFFFFFFF - ((0xFFFFFFFF - (pwm10_dev.gpt.tldr + 1)) / 2);
	pwm10_dev.gpt.tclr = DEFAULT_TCLR;

	sema_init(&pwm10_dev.sem, 1);

	memset(&pwm11_dev, 0, sizeof(struct pwm_dev));

	/* change these 4 values to use a different PWM */
	pwm11_dev.frequency = defaultFrequency;
	pwm11_dev.gpt.timer_num = 11;
	pwm11_dev.gpt.mux_offset = GPT11_MUX_OFFSET;
	pwm11_dev.gpt.gpt_base = PWM11_CTL_BASE;
	pwm11_dev.gpt.input_freq = CLK_32K_FREQ;

	pwm11_dev.gpt.tldr = 0xFFFFFFFF - (pwm11_dev.gpt.input_freq / pwm11_dev.frequency) + 1;
	pwm11_dev.gpt.tmar = 0xFFFFFFFF - ((0xFFFFFFFF - (pwm11_dev.gpt.tldr + 1)) / 2);
	pwm11_dev.gpt.tclr = DEFAULT_TCLR;

	sema_init(&pwm11_dev.sem, 1);

	if (pwm_init_cdev())
		goto init_fail;

	printk(KERN_INFO "OMAP3 PWM: character device initialized (major=%d)\n", MAJOR(devt));

	return 0;

init_fail:
	return -1;
}
module_init(pwm_init);

static void __exit pwm_exit(void)
{
	cdev_del(&cdev);

	pwm_off(&pwm9_dev);
	restore_mux(&pwm9_dev);

	if (pwm9_dev.user_buff)
		kfree(pwm9_dev.user_buff);

	pwm_off(&pwm10_dev);
	restore_mux(&pwm10_dev);

	if (pwm10_dev.user_buff)
		kfree(pwm10_dev.user_buff);

	pwm_off(&pwm11_dev);
	restore_mux(&pwm11_dev);

	if (pwm11_dev.user_buff)
		kfree(pwm11_dev.user_buff);

	unregister_chrdev_region(devt, 3);

}
module_exit(pwm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Scott Ellis - Jumpnow");
MODULE_AUTHOR("Steve Chang");
MODULE_DESCRIPTION("PWM example for Gumstix Overo"); 

