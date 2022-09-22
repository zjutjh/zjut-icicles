#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
};

static const struct modversion_info ____versions[]
__attribute_used__
__attribute__((section("__versions"))) = {
	{ 0x79021143, "struct_module" },
	{ 0x806d5133, "param_array_get" },
	{ 0x89cef6fb, "param_array_set" },
	{ 0x6483655c, "param_get_short" },
	{ 0x40046949, "param_set_short" },
	{ 0xd2f1de99, "i2c_register_driver" },
	{ 0x4ba89b2e, "malloc_sizes" },
	{ 0x237ea3cd, "device_create_file" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0xcd329670, "rtc_device_register" },
	{ 0xa31c4cd, "i2c_attach_client" },
	{ 0x73e20c1c, "strlcpy" },
	{ 0xaab785c8, "kmem_cache_zalloc" },
	{ 0xed584a39, "i2c_master_send" },
	{ 0x90b72f24, "seq_printf" },
	{ 0x1d26aa98, "sprintf" },
	{ 0xff178f6, "__aeabi_idivmod" },
	{ 0x2196324, "__aeabi_idiv" },
	{ 0xdd132261, "printk" },
	{ 0x127b48e8, "dev_driver_string" },
	{ 0xb4740049, "i2c_transfer" },
	{ 0xb3593a0f, "i2c_probe" },
	{ 0x37a0cba, "kfree" },
	{ 0x3c37dfa4, "i2c_detach_client" },
	{ 0x582bda37, "rtc_device_unregister" },
	{ 0xdce55970, "i2c_del_driver" },
};

static const char __module_depends[]
__attribute_used__
__attribute__((section(".modinfo"))) =
"depends=";

