#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/of_fdt.h>
#include <asm/setup.h>
#include <linux/seq_file.h>
#include <asm/setup.h>
#include <soc/qcom/bootinfo.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#ifndef CONFIG_ARM64
#include <asm/mach/arch.h>
#endif


#define UNLOCK_CODE_LEN 256
#define PUBK_SHA1_FP_SIZE 60
//#define DUMP_UNLOCK_CODE

static char unlock_code_string[2*UNLOCK_CODE_LEN + PUBK_SHA1_FP_SIZE + 1] = "\0";
struct proc_dir_entry * sec_unlock_entry = NULL;


#ifdef DUMP_UNLOCK_CODE
static void dump_unlock_code(char *src, char *dst, int src_len, int dst_len)
{

    int i = 0;
    char temp[3] = {0};

    if (dst_len <= src_len * 2) {
        pr_err("[Unlock BL]:%s\tinput length exceeds the output one\n", __func__);
        return;
    }

    for (i = 0; i < src_len; i++) {
        sprintf(temp, "%02x", (unsigned char)src[i]);
        memcpy(&dst[ i*2 ], temp, 2);
    }

    printk("[Unlock BL]:Dump Hex str %d length is:%s\n", (int)strlen(dst), dst);
    return;
}
#endif

ssize_t sec_unlock_proc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{

    int len = strlen(unlock_code_string);
#ifdef DUMP_UNLOCK_CODE
    char hex_buff[2*UNLOCK_CODE_LEN + 1] = {0};
#endif
    if (*f_pos > 0) {
        printk("[Unlock BL]: Need to locate correct position\n");
        *f_pos = 0;
        return -EINVAL;
    }

#ifdef DUMP_UNLOCK_CODE
    /* Note the original data in the unlock buffer where passed from LK is binary formatter.
     * Nevertheless it's without meaning since encrypted data may contain '\0' as terminator.
     */
    printk("[Unlock BL]: data from LK corruption with real len:(%d)\n", len);
    dump_unlock_code(unlock_code_string, hex_buff, UNLOCK_CODE_LEN, 2*UNLOCK_CODE_LEN +1);

#endif

    if (copy_to_user((char*)buf, unlock_code_string, len)) {
        pr_err("[Unlock_BL]: Copy data to userspace error\n");
        return -EFAULT;
    }

    *f_pos += len;
    return len;

}

ssize_t sec_unlock_proc_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    /* Dont allow to write or modification this memory. */
    return 0;
}

ssize_t sec_unlock_proc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t sec_unlock_proc_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);

static struct file_operations unlock_file_fops = {
        .read = sec_unlock_proc_read,
        .write = sec_unlock_proc_write,
};


static int __init unlock_bl_of_populate_setup(void)
{
    int ret = -1;
    struct device_node *n = of_find_node_by_path("/chosen");
    const char *unlock_code = NULL;

    ret = of_property_read_string(n, "mmi,unlock_code", &unlock_code);
    if (ret == 0) {
        strlcpy(unlock_code_string, unlock_code, sizeof(unlock_code_string));
    }

    of_node_put(n);

    return ret;
}


static int __init proc_unlock_init(void)
{
    int ret = 0;
    ret = unlock_bl_of_populate_setup();
    if (ret != 0) {

        pr_err("[Unlock BL]: Unable to acquire the mmi unlock_code\n\r");
        return -1;
    }


    sec_unlock_entry = proc_create("sec_unlock_code", 0664, NULL, &unlock_file_fops);
    if (sec_unlock_entry == NULL) {
        pr_err("[Unlock BL]: Unable to create /proc entry\n\r");
        return -1;
    }

    return 0;
}


static void __exit proc_unlock_exit(void) {

    if (NULL != sec_unlock_entry) {
        proc_remove(sec_unlock_entry);
    }
}


module_init(proc_unlock_init);
module_exit(proc_unlock_exit);
MODULE_DESCRIPTION("Lenovo Mobile Group. Unlock code");
MODULE_LICENSE("GPL v2");

