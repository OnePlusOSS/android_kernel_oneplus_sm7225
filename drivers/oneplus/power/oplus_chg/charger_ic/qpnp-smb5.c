#ifdef OEM_TARGET_PRODUCT_EBBA
#include <linux/module.h>

int __g_video_call_on;
module_param_named(
	video_call_on, __g_video_call_on, int, 0644
);
#endif
