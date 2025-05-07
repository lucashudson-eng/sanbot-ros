package com.grin.sanbotopensdkmqtt.video;


import android.util.Log;

import com.sanbot.opensdk.beans.OperationResult;
import com.sanbot.opensdk.function.beans.StreamOption;
import com.sanbot.opensdk.function.unit.HDCameraManager;
import com.sanbot.opensdk.function.unit.interfaces.media.MediaStreamListener;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by admin on 2016/6/13.
 */
public class VideoManager implements MediaStreamListener {

    private static final String TAG = VideoManager.class.getSimpleName();

    private HDCameraManager mediaManager;

    private VideoDataCallback callback;

    private static VideoManager instance;

    private List<Integer> handleList;


    public static VideoManager getVideoManager(HDCameraManager mediaManager) {
        if (instance == null) {
            synchronized (VideoManager.class) {
                if (instance == null) {
                    instance = new VideoManager(mediaManager);
                }
            }
        }
        return instance;
    }

    public VideoManager(HDCameraManager mediaManager) {
        this.mediaManager = mediaManager;
        this.mediaManager.setMediaListener(this);
        handleList = new ArrayList<>();
    }

    public void setVideoDataCallback(VideoDataCallback callback) {
        this.callback = callback;
    }

    public int startVideo() {
        StreamOption streamOption = new StreamOption();
        streamOption.setChannel(StreamOption.MAIN_STREAM);
        streamOption.setDecodType(StreamOption.HARDWARE_DECODE);
        streamOption.setJustIframe(false);
        OperationResult operationResult = mediaManager.openStream(streamOption);
        String result = operationResult.getResult();
        int resultCode = Integer.valueOf(result);
        Log.i(TAG, "startVideo: result=" + result);
        if (resultCode == -1) {
            return -1;
        }
        handleList.add(resultCode);
        return resultCode;
    }

    public void closeAllLive() {
        for (Integer item : handleList) {
            Log.d(TAG, "handle : " + item + "关闭：" + mediaManager.closeStream(item).getResult());
        }
        handleList.clear();
        instance = null;
    }

    public void closeLive(int handle) {
        Log.d(TAG, "关闭：" + mediaManager.closeStream(handle));
    }


    @Override
    public void getAudioStream(int handle, byte[] bytes) {

    }

    @Override
    public void getVideoStream(int handle, byte[] bytes, int width, int height) {
        if (callback != null) {
            callback.onYUVBack(handle, bytes, width, height);
        }
    }
}
