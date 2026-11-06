package com.orbbec.unityadapt;

/** Minimal stub so libOrbbecNative JNI_OnLoad can resolve method IDs. */
public class CameraNative {
    public static CameraNative Instance;

    public static void nativeCallStartPreview() {
        // no-op for M0 probe
    }

    public static void nativeCallStopPreview() {
        // no-op for M0 probe
    }
}
