package com.orbbec.unityadapt;

/**
 * JNI surface implemented by BraveJelly {@code libOrbbecNative.so}.
 * {@link #OnGetPermission} registers the Android UsbManager FD for libusb/OpenNI.
 */
public final class CallbackAdapt {
    private CallbackAdapt() {}

    public static native int OnGetPermission(int vendorId, int productId, int fileDescriptor,
                                             int busNum, int devNum, String usbFsRoot);

    public static native int init();

    public static native int debug(String msg);
}
