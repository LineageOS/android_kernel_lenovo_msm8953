#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/sched.h>

// ring buffer size is 512K
#define PERFTAGS_BUFFER_SIZE	(512 * 1024)
#define PERFTAGS_MAX_LINE_SIZE  256
#define PERFTAGS_TS_LENGTH      40

static unsigned char perftags_initialized = 0;
static int perftags_fops_open(struct inode* inode, struct file* filp);
static int perftags_fops_release(struct inode* inode, struct file* filp);
static ssize_t perftags_fops_read(struct file* filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t perftags_fops_write(struct file* filp, const char __user *buf, size_t count, loff_t *f_pos);

struct perftags_context_t {
	struct mutex lock;
	void*        buffer;
	size_t       size;
	size_t       offset;
};

static struct perftags_context_t* perftags_context = NULL;

static struct file_operations perftags_fops = {
	.owner = THIS_MODULE,
	.open = perftags_fops_open,
	.release = perftags_fops_release,
	.read = perftags_fops_read,
	.write = perftags_fops_write,
};

static struct miscdevice perftags_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "perftags",
	.fops  = &perftags_fops,
};

static int perftags_fops_open(struct inode* inode, struct file* filp)
{
	return 0;
}

static int perftags_fops_release(struct inode* inode, struct file* filp)
{
	return 0;
}

static const char* perftags_get_local_time(char* buf)
{
	struct timex tx;
	struct rtc_time tm;

	// get UTC time
	do_gettimeofday(&tx.time);

	// convert to local time
	tx.time.tv_sec -= sys_tz.tz_minuteswest * 60;

	// compute rtc time
	rtc_time_to_tm(tx.time.tv_sec, &tm);

	// format
	sprintf(buf, "%02d-%02d %02d:%02d:%02d.%03d",
		tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec,
		(int)(tx.time.tv_usec / 1000));

	return buf;
}

static ssize_t perftags_do_write(const char* buf, size_t count, int is_user)
{
	char line_buf[PERFTAGS_MAX_LINE_SIZE];
	int line_len = 0;
	unsigned long ts_usec = 0;
	int blk_len = 0;
	int offset = 0;
	char ts[32] = {0};

	// get timestamp first
	ts_usec = ((unsigned long)local_clock()) / 1000;

	if (!perftags_initialized) {
		return -1;
	}

	// check to avoid buffer overflow
	if (count > (PERFTAGS_MAX_LINE_SIZE - PERFTAGS_TS_LENGTH)) {
		return -1;
	}

	// protected by mutex lock
	mutex_lock(&perftags_context->lock);

	// save timestamp
	line_len = sprintf(line_buf, "%lu.%06d %s ",
		(unsigned long)(ts_usec / 1000000), (int)(ts_usec % 1000000),
		perftags_get_local_time(ts));

	// copy data
	if (is_user) {
		if (copy_from_user(line_buf + line_len, buf, count) != 0) {
			mutex_unlock(&perftags_context->lock);
			return -1;
		}
	} else {
		memcpy(line_buf + line_len, buf, count);
	}
	line_len += count;

	// append '\n'
	if (line_buf[line_len - 1] != '\n') {
		line_buf[line_len++] = '\n';
	}

	// save to ring buffer
	blk_len = PERFTAGS_BUFFER_SIZE - perftags_context->offset;
	if (line_len > blk_len) {
		offset = line_len - blk_len;
		memcpy(perftags_context->buffer + perftags_context->offset, line_buf,
			blk_len);
		memcpy(perftags_context->buffer, line_buf + blk_len, line_len - blk_len);
	} else {
		offset = perftags_context->offset + line_len;
		memcpy(perftags_context->buffer + perftags_context->offset, line_buf,
			line_len);
	}

	// update offset, size
	perftags_context->offset = offset;
	if (perftags_context->size < PERFTAGS_BUFFER_SIZE) {
		perftags_context->size += line_len;
		if (perftags_context->size > PERFTAGS_BUFFER_SIZE) {
			perftags_context->size = PERFTAGS_BUFFER_SIZE;
		}
	}

	mutex_unlock(&perftags_context->lock);

	return count;
}

ssize_t perftags_write(const char* buf)
{
	if (buf && *buf) {
		return perftags_do_write(buf, strlen(buf), 0);
	}

	return 0;
}
EXPORT_SYMBOL(perftags_write);

static ssize_t perftags_fops_read(struct file* filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int cnt = 0;

	mutex_lock(&perftags_context->lock);

	// check rest data size
	cnt = perftags_context->size - (*f_pos);
	if (cnt <= 0) {
		mutex_unlock(&perftags_context->lock);
		return 0;
	}

	// check user buffer size
	if (cnt > count) {
		cnt = count;
	}

	// copy data
	if (copy_to_user(buf, perftags_context->buffer + (*f_pos), cnt)) {
		pr_err("[%s]: fail to copy to user\n", __func__);
		mutex_unlock(&perftags_context->lock);
		return -1;
	}
	*f_pos += cnt;
	mutex_unlock(&perftags_context->lock);

	return cnt;
}


static ssize_t perftags_fops_write(struct file* filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	if (perftags_do_write(buf, count, 1) < 0) {
		return -1;
	}

	*f_pos += count;

	return count;
}

static int perftags_init(void)
{
    pr_info("[%s]: enter\n", __func__);

    // prepare context
    perftags_context = (struct perftags_context_t*)kmalloc(sizeof(struct perftags_context_t),
    	GFP_KERNEL);
    if (NULL == perftags_context) {
    	pr_err("[%s]: fail to allocate memory\n", __func__);
    	return -ENOMEM;
    }

    // prepare buffer
    perftags_context->buffer = kmalloc(PERFTAGS_BUFFER_SIZE, GFP_KERNEL);
    if (NULL == perftags_context->buffer) {
    	pr_err("[%s]: fail to assign buffer\n", __func__);
    	kfree(perftags_context);
    	return -ENOMEM;
    }

    // initialize context
    mutex_init(&perftags_context->lock);
    perftags_context->size = 0;
    perftags_context->offset = 0;

    // create misc device
    if (misc_register(&perftags_dev)) {
    	pr_err("[%s]: fail to register misc device\n", __func__);
    	kfree(perftags_context->buffer);
    	kfree(perftags_context);
    	return -EAGAIN;
    }

    pr_info("[%s]: successfully\n", __func__);
	perftags_initialized = 1;

    return 0;
}

static void perftags_exit(void)
{
    pr_info("[%s]: enter\n", __func__);

    // cleanup
	perftags_initialized = 0;
    misc_deregister(&perftags_dev);
    kfree(perftags_context->buffer);
    kfree(perftags_context);

    pr_info("[%s]: exit\n", __func__);
}

MODULE_AUTHOR("Lenovo");
MODULE_DESCRIPTION("performace tags");
MODULE_LICENSE("GPL");

module_init(perftags_init);
module_exit(perftags_exit);

