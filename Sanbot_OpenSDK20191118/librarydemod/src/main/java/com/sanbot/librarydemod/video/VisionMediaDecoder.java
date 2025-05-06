package com.sanbot.librarydemod.video;


import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaFormat;
import android.util.Log;
import android.view.Surface;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * 负责视频解码和显示
 */
public class VisionMediaDecoder {
    private String TAG = "解码器：";

    /**
     * 解码超时时间
     */
    private long decodeTimeout = 10000;

    /**
     * lock.
     */
    private Lock lock = new ReentrantLock();

    /**
     * 相关的视频显示迿?
     */
    private Surface surface;

    /**
     * 视频信息.
     */
    private MediaCodec.BufferInfo videoBufferInfo = new MediaCodec.BufferInfo();

    /**
     * 视频解码器
     */
    private MediaCodec videoDecoder;

    /**
     * 视频高度.
     */
    private int videoHeight = 0;

    /**
     * 当前视频缓冲匿
     */
    private ByteBuffer[] videoInputBuffers;

    /**
     * 当前视频浿MIME 类型
     */
    private String videoMimeType = "video/avc";

    /**
     * 视频width
     */
    private int videoWidth = 0;

    /**
     * 解码并显显示视频流
     *
     * @param
     * @param
     */
    public void drawVideoSample(ByteBuffer sampleData) {
        try {
            lock.lock();
            if (videoDecoder == null) {
                return;
            }
            // put sample data
            int inIndex = videoDecoder.dequeueInputBuffer(decodeTimeout);
            if (inIndex >= 0) {
                ByteBuffer buffer = videoInputBuffers[inIndex];
                int sampleSize = sampleData.limit();
                buffer.clear();
                buffer.put(sampleData);
                buffer.flip();
                // Log.i("DecodeActivity", "" + buffer.toString());
                Log.i(TAG, "showVideo: 获取到视频指针位置！");
                videoDecoder.queueInputBuffer(inIndex, 0, sampleSize, 0, 0);
            }
            // output, 1 microseconds = 100,0000 / 1 second
            int ret = videoDecoder.dequeueOutputBuffer(videoBufferInfo,
                    decodeTimeout);
            int size = videoBufferInfo.size;
            if (ret < 0) {
                onDecodingError(ret);
                return;
            }
            videoDecoder.releaseOutputBuffer(ret, true);
        } catch (Exception e) {
            Log.e(TAG, e.getMessage(), e);
            stopDecoding();
        } finally {
            lock.unlock();
        }

    }

    /**
     * @return the surface
     */
    public Surface getSurface() {
        return surface;
    }

    /**
     * @return the videoHeight
     */
    public int getVideoHeight() {
        return videoHeight;
    }

    /**
     * @return the videoWidth
     */
    public int getVideoWidth() {
        return videoWidth;
    }

    private void onDecodingError(int index) {
        switch (index) {
            case MediaCodec.INFO_OUTPUT_BUFFERS_CHANGED:
                Log.e(TAG, "onDecodingError: The output buffers have changed");
                // The output buffers have changed, the client must refer to the
                // new
                // set of output buffers returned by getOutputBuffers() from
                // this
                // point on.
                // outputBuffers = decoder.getOutputBuffers();
                break;

            case MediaCodec.INFO_OUTPUT_FORMAT_CHANGED:
                Log.d(TAG, "New format: " + videoDecoder.getOutputFormat());
                // The output format has changed, subsequent data will follow
                // the
                // new format. getOutputFormat() returns the new format.
                break;

            case MediaCodec.INFO_TRY_AGAIN_LATER:
                Log.d(TAG, "dequeueOutputBuffer timed out!");
                // If a non-negative timeout had been specified in the call to
                // dequeueOutputBuffer(MediaCodec.BufferInfo, long), indicates
                // that
                // the call timed out.
                break;

            default:
                break;
        }
    }

    /**
     * 当碰到媒体流的同步糥 即关键帧 .
     *
     * @param
     */
    public void onCreateCodec(int width, int height) {
        setVideoWidth(width);
        setVideoHeight(height);
        stopDecoding();
        // 初始化解码器
        startDecoding();
    }

    /**
     * @param surface the surface to set
     */
    public void setSurface(Surface surface) {
        this.surface = surface;
    }

    /**
     * @param videoHeight the videoHeight to set
     */
    public void setVideoHeight(int videoHeight) {
        this.videoHeight = videoHeight;
    }

    /**
     * @param videoWidth the videoWidth to set
     */
    public void setVideoWidth(int videoWidth) {
        this.videoWidth = videoWidth;
    }

    /**
     * 创建解码噿
     */
    public boolean startDecoding() {
        try {
            lock.lock();
            if (videoInputBuffers != null) {
                Log.w(TAG, "startDecoding: videoInputBuffers already created!");
                return false;
            } else if (videoDecoder != null) {
                Log.w(TAG, "startDecoding: videoDecoder already created!");
                return false;
            }
            MediaFormat format = MediaFormat.createVideoFormat(videoMimeType, getVideoWidth(), getVideoHeight());
            Log.i(TAG, "format:" + format);
            format.setInteger(MediaFormat.KEY_AAC_PROFILE, MediaCodecInfo.CodecProfileLevel.AACObjectLC);
            format.setInteger(MediaFormat.KEY_SAMPLE_RATE, 44100);
            format.setInteger(MediaFormat.KEY_CHANNEL_COUNT, 1);
            format.setInteger(MediaFormat.KEY_BIT_RATE, 64000);
            try {
                videoDecoder = MediaCodec.createDecoderByType(videoMimeType);
                videoDecoder.configure(format, getSurface(), null, 0);
                videoDecoder.start();
            } catch (IOException | IllegalStateException e) {
                e.printStackTrace();
            }
            videoInputBuffers = videoDecoder.getInputBuffers();
        } finally {
            Log.e("CODEC", "onCreateCodec");
            lock.unlock();
        }
        return true;
    }

    /**
     * 停止解码.
     */
    public void stopDecoding() {
        try {
            lock.lock();
            if (videoDecoder != null) {
                videoDecoder.stop();
                videoDecoder.release();
                videoDecoder = null;

                Log.i(TAG, "stopDecoding");
            }
            videoInputBuffers = null;
        } finally {
            Log.e("CODEC", "stopDecoding");
            lock.unlock();
        }
    }

}