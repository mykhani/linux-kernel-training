#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
/* for struct timeval */
#include <uapi/linux/time.h>
#include <linux/utsname.h>

/* Add your code here */
char *whom;
char *version;

struct timeval time;

unsigned long long start, end;

extern struct uts_namespace init_uts_ns;

static int __init hello_init(void)
{
	do_gettimeofday(&time);
	start = time.tv_sec;
	
	printk("Hello %s. You are currently using Linux %s \r\n", whom, init_uts_ns.name.release);
	return 0;
}

static void __exit hello_exit(void)
{
	do_gettimeofday(&time);
	end = time.tv_sec;
	printk("Byte bye %s \r\n", whom);
	printk("Time since module loaded %llu seconds \r\n", (end - start));
	return;
}

module_param(whom, charp, 0);

module_init(hello_init);
module_exit(hello_exit);
MODULE_AUTHOR("Yasir Khan <yasir_khan@mentor.com>");

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A simple module which takes character string as parameter and prints the Linux kernel version");

