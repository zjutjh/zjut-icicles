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
	{ 0xd2f1de99, "i2c_register_driver" },
	{ 0x4ba89b2e, "malloc_sizes" },
	{ 0x9ad8806a, "class_device_create" },
	{ 0x66a4982f, "class_create" },
	{ 0xd05d837c, "register_chrdev" },
	{ 0xa31c4cd, "i2c_attach_client" },
	{ 0xe914e41e, "strcpy" },
	{ 0xaab785c8, "kmem_cache_zalloc" },
	{ 0x98082893, "__copy_to_user" },
	{ 0xb3593a0f, "i2c_probe" },
	{ 0x37a0cba, "kfree" },
	{ 0x3c37dfa4, "i2c_detach_client" },
	{ 0xc192d491, "unregister_chrdev" },
	{ 0x843af3ad, "class_destroy" },
	{ 0x67010cbf, "class_device_destroy" },
	{ 0xdce55970, "i2c_del_driver" },
	{ 0xdd132261, "printk" },
};

static const char __module_depends[]
__attribute_used__
__attribute__((section(".modinfo"))) =
"depends=";

