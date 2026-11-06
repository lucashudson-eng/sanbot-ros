package com.grin.sanbotrosbridge;

import com.orbbec.unityadapt.CallbackAdapt;

/**
 * Loads Orbbec/OpenNI natives (from BraveJelly-order deps) and exposes frame grab for body RTMP.
 */
final class OrbbecNative {
    static {
        System.loadLibrary("usb100");
        System.loadLibrary("orbbecusb");
        System.loadLibrary("jpeg-turbo1500");
        System.loadLibrary("uvc");
        System.loadLibrary("UVCCamera");
        System.loadLibrary("mpbase");
        System.loadLibrary("gesture");
        System.loadLibrary("XnCore");
        System.loadLibrary("XnFormats");
        System.loadLibrary("XnDDK");
        System.loadLibrary("XnDeviceFile");
        System.loadLibrary("OpenNI");
        System.loadLibrary("XnDeviceSensorV2");
        System.loadLibrary("OrbbecNative");
        System.loadLibrary("orbbec_body");
    }

    private OrbbecNative() {}

    static int onGetPermission(int vendorId, int productId, int fileDescriptor,
                               int busNum, int devNum, String usbFsRoot) {
        return CallbackAdapt.OnGetPermission(
                vendorId, productId, fileDescriptor, busNum, devNum, usbFsRoot);
    }

    static native String nativeOpen(String xmlPath);

    static native void nativeClose();

    /** int[3] = { width, height, useColorFlag } */
    static native int[] nativeFrameSize();

    /** Fill ARGB ints (length >= w*h). Returns 1 on success. Color if available, else depth-gray. */
    static native int nativeGrabArgb(int[] argb);

    /** Always fill from depth map as grayscale ARGB (even when color session is open). */
    static native int nativeGrabDepthArgb(int[] argb);
}
