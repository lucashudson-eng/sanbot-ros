package com.grin.sanbotopensdkmqtt.video;

/**
 * Created by admin on 2017/7/6.
 */
public interface VideoDataCallback {
    void onYUVBack(int handle, byte[] buf, int width, int height);
}
