OnePlus NORD CE Oxygen OS 11.0.5.5EB13AA :

If you build using aosp commands, before setting up environment 

In "android/vendor/qcom/proprietary/prebuilt_grease/vendorsetup.sh" 
comment out below line as shown((echo is added to display message))

if [ -e "$dest" ];then
+echo "*****dont restore/delete symlink*******"
+#rm -rf $dest
fi

and add below in "android/build/core/main.mk" to build boot and dtbo together

PHONY: bootimage
bootimage: $(INSTALLED_BOOTIMAGE_TARGET) \
+           $(INSTALLED_DTBOIMAGE_TARGET)

then do

1. source build/envsetup.sh

2. lunch (choose option 29 lito-userdebug)

3. ./build.sh target_only bootimage


Synchronize codes for OnePlus NORD CE Oxygen OS 11.0.6.6EB13BA
=======================
- Kernel has no update -


Synchronize codes for OnePlus NORD CE Oxygen OS 11.0.12.12EB13DA
=======================
---- Kernel update ----

1. Submit the Google security patch CVE-2021-30306 for 2021-10

2. Fix ->session locking

3. fix a kernel-infoleak in qrtr_recvmsg()


Synchronize codes for OnePlus NORD CE Oxygen OS 11.0.14.14EB13BA
=======================
---- Kernel update ----

1. Submit the Google security patch for 2021-12

2. Submit the Google security patch for 2022-01



