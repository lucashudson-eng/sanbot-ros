package com.qihan.HumanFollow.video;

import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaFormat;
import android.util.Log;
import android.view.Surface;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/** H.264 decode of Sanbot HDCameraManager stream onto a Surface. */
public class VisionMediaDecoder {
    private static final String TAG = "HumanFollow-decoder";

    private final Lock lock = new ReentrantLock();
    private long decodeTimeout = 10000;
    private Surface surface;
    private final MediaCodec.BufferInfo videoBufferInfo = new MediaCodec.BufferInfo();
    private MediaCodec videoDecoder;
    private int videoHeight;
    private int videoWidth;
    private ByteBuffer[] videoInputBuffers;
    private final String videoMimeType = "video/avc";

    public void drawVideoSample(ByteBuffer sampleData) {
        try {
            lock.lock();
            if (videoDecoder == null) {
                return;
            }
            int inIndex = videoDecoder.dequeueInputBuffer(decodeTimeout);
            if (inIndex >= 0) {
                ByteBuffer buffer = videoInputBuffers[inIndex];
                int sampleSize = sampleData.limit();
                buffer.clear();
                buffer.put(sampleData);
                buffer.flip();
                videoDecoder.queueInputBuffer(inIndex, 0, sampleSize, 0, 0);
            }
            int ret = videoDecoder.dequeueOutputBuffer(videoBufferInfo, decodeTimeout);
            if (ret < 0) {
                return;
            }
            videoDecoder.releaseOutputBuffer(ret, true);
        } catch (Exception e) {
            Log.e(TAG, "drawVideoSample", e);
            stopDecoding();
        } finally {
            lock.unlock();
        }
    }

    public Surface getSurface() {
        return surface;
    }

    public int getVideoHeight() {
        return videoHeight;
    }

    public int getVideoWidth() {
        return videoWidth;
    }

    public void onCreateCodec(int width, int height) {
        setVideoWidth(width);
        setVideoHeight(height);
        stopDecoding();
        startDecoding();
    }

    public void setSurface(Surface surface) {
        this.surface = surface;
    }

    public void setVideoHeight(int videoHeight) {
        this.videoHeight = videoHeight;
    }

    public void setVideoWidth(int videoWidth) {
        this.videoWidth = videoWidth;
    }

    public boolean startDecoding() {
        try {
            lock.lock();
            if (videoInputBuffers != null || videoDecoder != null) {
                return false;
            }
            MediaFormat format = MediaFormat.createVideoFormat(
                    videoMimeType, getVideoWidth(), getVideoHeight());
            format.setInteger(MediaFormat.KEY_AAC_PROFILE, MediaCodecInfo.CodecProfileLevel.AACObjectLC);
            format.setInteger(MediaFormat.KEY_SAMPLE_RATE, 44100);
            format.setInteger(MediaFormat.KEY_CHANNEL_COUNT, 1);
            format.setInteger(MediaFormat.KEY_BIT_RATE, 64000);
            try {
                videoDecoder = MediaCodec.createDecoderByType(videoMimeType);
                videoDecoder.configure(format, getSurface(), null, 0);
                videoDecoder.start();
            } catch (IOException | IllegalStateException e) {
                Log.e(TAG, "startDecoding configure", e);
            }
            if (videoDecoder != null) {
                videoInputBuffers = videoDecoder.getInputBuffers();
            }
        } finally {
            lock.unlock();
        }
        return true;
    }

    public void stopDecoding() {
        try {
            lock.lock();
            if (videoDecoder != null) {
                try {
                    videoDecoder.stop();
                } catch (Exception ignored) {
                }
                try {
                    videoDecoder.release();
                } catch (Exception ignored) {
                }
                videoDecoder = null;
            }
            videoInputBuffers = null;
        } finally {
            lock.unlock();
        }
    }
}
