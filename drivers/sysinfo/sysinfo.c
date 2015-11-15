/*
 * Sample kobject implementation
 *
 * Copyright (C) 2004-2007 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (C) 2007 Novell Inc.
 *
 * Released under the GPL version 2 only.
 *
 */

#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>	
#include <linux/seq_file.h>
#include <../gpio-names.h>

#define KAREN_KERNEL_VERSION "K01.33.20131113"
#define MANUFACTURER  "Wistron"
const char *pcb_ver_name[4]={"SA","SB","SC","-1"};

extern char boot_ver[16];
extern int pcb_ver;
extern int cpuid[17];
#ifdef CONFIG_TOUCHSCREEN_FTS
extern int fts_proc_create_touch();
#endif

static struct proc_dir_entry *proc_sysinfo_dir;	

static int proc_bootloader_ver_show(struct seq_file *m, void *v)
{	
    seq_printf(m, "%s\n" , boot_ver);	
	return 0;	
}

static int proc_kernel_ver_show(struct seq_file *m, void *v)
{	
    seq_printf(m, "%s\n" , KAREN_KERNEL_VERSION);	
	return 0;	
}

static int proc_pcb_ver_show(struct seq_file *m, void *v)
{	
    seq_printf(m, "%s\n" , pcb_ver_name[pcb_ver]);	
	return 0;	
}

static int proc_wcis_show(struct seq_file *m, void *v)
{	
     mm_segment_t fs;
     struct file *fp;
     char wcis[2]={0};

     fs = get_fs();
     set_fs(get_ds());
     fp = filp_open("/sys/bus/i2c/devices/4-0054/wcis" ,O_RDONLY ,0 );
     if(fp != NULL)
     {
         fp->f_op->read(fp, wcis, 1, &fp->f_pos);
     }
     else
     {
         printk("open file fail!\n");
     }
     set_fs(fs);
     if(wcis[0]!='1')
         wcis[0]='0';
     seq_printf(m, "%s\n" , wcis);
     filp_close(fp, NULL);
     return 0;	
}

static int proc_os_version_show(struct seq_file *m, void *v)
{	
    seq_printf(m, "%d\n" , KAREN_OS_VERSION);	
	return 0;	
}

static int proc_sn_show(struct seq_file *m, void *v)
{	
     mm_segment_t fs;
     struct file *fp;
     char sn[23]={0};

     fs = get_fs();
     set_fs(get_ds());
     fp = filp_open("/sys/bus/i2c/devices/4-0054/sn" ,O_RDONLY ,0 );
     if(fp != NULL)
     {
         fp->f_op->read(fp, sn, 22, &fp->f_pos);
     }
     else
     {
         printk("open file fail!\n");
     }
     set_fs(fs);
     seq_printf(m, "%s\n" , sn);
     filp_close(fp, NULL);
     return 0;	
}

static int proc_manufacturer_show(struct seq_file *m, void *v)
{	
    seq_printf(m, "%s\n" , MANUFACTURER);	
	return 0;	
}

static int proc_cpuid_show(struct seq_file *m, void *v)
{	
    seq_printf(m, "%s\n" , cpuid);	
	return 0;	
}

static int open_bootloader_ver_show(struct inode *inode, struct file *file)	
{	
	return single_open(file, proc_bootloader_ver_show, NULL);	
}

static int open_kernel_ver_show(struct inode *inode, struct file *file)	
{	
	return single_open(file, proc_kernel_ver_show, NULL);
}

static int open_pcb_ver_show(struct inode *inode, struct file *file)	
{	
	return single_open(file, proc_pcb_ver_show, NULL);
}

static int open_wcis_show(struct inode *inode, struct file *file)	
{	
	return single_open(file, proc_wcis_show, NULL);
}

static int open_os_version_show(struct inode *inode, struct file *file)	
{	
	return single_open(file, proc_os_version_show, NULL);
}

static int open_sn_show(struct inode *inode, struct file *file)	
{	
	return single_open(file, proc_sn_show, NULL);
}

static int open_manufacturer_show(struct inode *inode, struct file *file)	
{	
	return single_open(file, proc_manufacturer_show, NULL);
}

static int open_cpuid_show(struct inode *inode, struct file *file)	
{	
	return single_open(file, proc_cpuid_show, NULL);
}

static const struct file_operations proc_bootloader_ver_operations = {	
    .open       = open_bootloader_ver_show,	
    .read       = seq_read,	
    .llseek     = seq_lseek,	
    .release    = seq_release,	
};

static const struct file_operations proc_kernel_ver_operations = {	
    .open       = open_kernel_ver_show,	
    .read       = seq_read,	
    .llseek     = seq_lseek,	
    .release    = seq_release,	
};

static const struct file_operations proc_pcb_ver_operations = {	
    .open       = open_pcb_ver_show,	
    .read       = seq_read,	
    .llseek     = seq_lseek,	
    .release    = seq_release,	
};

static const struct file_operations proc_wcis_operations = {	
    .open       = open_wcis_show,	
    .read       = seq_read,	
    .llseek     = seq_lseek,	
    .release    = seq_release,	
};

static const struct file_operations proc_os_version_operations = {	
    .open       = open_os_version_show,	
    .read       = seq_read,	
    .llseek     = seq_lseek,	
    .release    = seq_release,	
};

static const struct file_operations proc_sn_operations = {	
    .open       = open_sn_show,	
    .read       = seq_read,	
    .llseek     = seq_lseek,	
    .release    = seq_release,	
};

static const struct file_operations proc_manufacturer_operations = {	
    .open       = open_manufacturer_show,	
    .read       = seq_read,	
    .llseek     = seq_lseek,	
    .release    = seq_release,	
};

static const struct file_operations proc_cpuid_operations = {	
    .open       = open_cpuid_show,	
    .read       = seq_read,	
    .llseek     = seq_lseek,	
    .release    = seq_release,	
};

static int sysinfo_init(void)
{
	          
        proc_sysinfo_dir = proc_mkdir("sysinfo", NULL);
	//sysinfo members
	proc_create("sysinfo/bootloader_ver", 0, NULL, &proc_bootloader_ver_operations);	
        proc_create("sysinfo/kernel_ver", 0, NULL, &proc_kernel_ver_operations);
        proc_create("sysinfo/pcb_ver", 0, NULL, &proc_pcb_ver_operations);
        proc_create("sysinfo/WCIS", 0, NULL, &proc_wcis_operations);
        proc_create("sysinfo/os_version", 0, NULL, &proc_os_version_operations);
        proc_create("sysinfo/sn", 0, NULL, &proc_sn_operations);
        proc_create("sysinfo/manufacturer", 0, NULL, &proc_manufacturer_operations);
        proc_create("sysinfo/CPUID", 0, NULL, &proc_cpuid_operations);
#ifdef CONFIG_TOUCHSCREEN_FTS	
	    fts_proc_create_touch();	
#endif
	return 0;
}

static void sysinfo_exit(void)
{
	remove_proc_entry("sysinfo", NULL);
}	

module_init(sysinfo_init);
module_exit(sysinfo_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Greg Kroah-Hartman <greg@kroah.com>");
