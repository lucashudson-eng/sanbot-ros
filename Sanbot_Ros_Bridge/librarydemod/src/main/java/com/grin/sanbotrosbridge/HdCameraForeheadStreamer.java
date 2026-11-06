package com.grin.sanbotrosbridge;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import com.pedro.common.ConnectChecker;
import com.pedro.encoder.input.sources.audio.NoAudioSource;
import com.pedro.encoder.input.sources.video.BufferVideoSource;
import com.pedro.library.generic.GenericStream;
import com.sanbot.opensdk.beans.OperationResult;
import com.sanbot.opensdk.function.beans.StreamOption;
import com.sanbot.opensdk.function.unit.HDCameraManager;
import com.sanbot.opensdk.function.unit.interfaces.media.MediaStreamListener;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

/**
 * OpenSDK {@link HDCameraManager} → still spike + optional RTMP {@code live/forehead}.
 * Phase 0: MAIN/SUB stills + HAL Camera0 still for comparison.
 * Phase 1: stream ARGB frames from {@code getVideoImage()} when distinct.
 */
final class HdCameraForeheadStreamer implements ConnectChecker {
    interface Listener {
        void onHdLog(String message);

        void onForeheadAvailabilityChanged(boolean available);
    }

    private static final String TAG = "HdForehead";
    private static final int STREAM_W = 640;
    private static final int STREAM_H = 480;
    private static final int STREAM_BITRATE = 800_000;

    private final Context appContext;
    private final Listener listener;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final AtomicBoolean publishing = new AtomicBoolean(false);
    private final AtomicBoolean healthy = new AtomicBoolean(false);
    private final AtomicBoolean startRequested = new AtomicBoolean(false);

    private HDCameraManager hdCameraManager;
    private GenericStream rtmpStream;
    private BufferVideoSource bufferSource;
    private Thread streamThread;
    private String brokerIp = "192.168.1.4";
    private int streamHandle = -1;
    private volatile boolean lastSpikeDistinct = false;
    private volatile boolean lastSpikeOk = false;
    private String lastSpikeSummary = "not run";

    HdCameraForeheadStreamer(Context context, Listener listener) {
        this.appContext = context.getApplicationContext();
        this.listener = listener;
    }

    void setHdCameraManager(HDCameraManager manager) {
        this.hdCameraManager = manager;
    }

    void setBrokerIp(String ip) {
        if (ip != null && !ip.trim().isEmpty()) {
            brokerIp = ip.trim();
        }
    }

    boolean isHealthy() {
        return healthy.get();
    }

    boolean isSpikeOk() {
        return lastSpikeOk;
    }

    boolean isLikelyDistinct() {
        return lastSpikeDistinct;
    }

    String getSpikeSummary() {
        return lastSpikeSummary;
    }

    /**
     * Phase 0: open MAIN then SUB, save JPEGs; optional HAL Camera 0 still.
     * Writes under {@code /sdcard/Download/}.
     * Prefer {@code includeHalFace=false} when Face RTMP may start — Nano cannot share camera 0.
     */
    void runSpikeAsync() {
        runSpikeAsync(true);
    }

    void runSpikeAsync(boolean includeHalFace) {
        new Thread(() -> runSpike(includeHalFace), "hdcam-spike").start();
    }

    private void runSpike() {
        runSpike(true);
    }

    private void runSpike(boolean includeHalFace) {
        File dir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        if (dir == null) {
            dir = appContext.getExternalFilesDir(null);
        }
        if (dir != null && !dir.exists()) {
            //noinspection ResultOfMethodCallIgnored
            dir.mkdirs();
        }
        StringBuilder summary = new StringBuilder();
        boolean mainOk = false;
        boolean subOk = false;
        boolean faceOk = false;
        int mainW = 0, mainH = 0, subW = 0, subH = 0, faceW = 0, faceH = 0;
        long mainBytes = 0, subBytes = 0, faceBytes = 0;

        log("HD spike start → " + (dir != null ? dir.getAbsolutePath() : "?"));

        // HDCamera first — HAL Camera.open(0) can block forever if Face RTMP owns the device.
        if (hdCameraManager == null) {
            lastSpikeOk = false;
            lastSpikeDistinct = false;
            lastSpikeSummary = "hdCameraManager=null";
            log("FAIL: HDCameraManager not bound (MainService?)");
            setHealthy(false);
            return;
        }

        File mainFile = dir != null ? new File(dir, "hdcam-main.jpg") : null;
        File subFile = dir != null ? new File(dir, "hdcam-sub.jpg") : null;
        mainOk = captureHdStill(StreamOption.MAIN_STREAM, mainFile);
        if (mainOk && mainFile != null && mainFile.isFile()) {
            mainBytes = mainFile.length();
            int[] wh = readJpegSize(mainFile);
            mainW = wh[0];
            mainH = wh[1];
            summary.append("main=").append(mainW).append('x').append(mainH)
                    .append(" bytes=").append(mainBytes).append("; ");
        } else {
            summary.append("main=FAIL; ");
        }

        subOk = captureHdStill(StreamOption.SUB_STREAM, subFile);
        if (subOk && subFile != null && subFile.isFile()) {
            subBytes = subFile.length();
            int[] wh = readJpegSize(subFile);
            subW = wh[0];
            subH = wh[1];
            summary.append("sub=").append(subW).append('x').append(subH)
                    .append(" bytes=").append(subBytes).append("; ");
        } else {
            summary.append("sub=FAIL; ");
        }

        if (includeHalFace) {
            faceOk = captureHalFaceStillTimed(dir != null ? new File(dir, "hdcam-face-hal.jpg") : null, 8_000);
            if (faceOk && dir != null) {
                File f = new File(dir, "hdcam-face-hal.jpg");
                faceBytes = f.length();
                int[] wh = readJpegSize(f);
                faceW = wh[0];
                faceH = wh[1];
                summary.append("face_hal=").append(faceW).append('x').append(faceH)
                        .append(" bytes=").append(faceBytes).append("; ");
            } else {
                summary.append("face_hal=FAIL; ");
            }
        } else {
            summary.append("face_hal=SKIPPED; ");
        }

        lastSpikeOk = mainOk || subOk;
        // Heuristic: distinct if HD still exists and resolution differs from HAL face,
        // or file size differs by >25%, or main!=sub resolution.
        boolean resDiff = (mainOk && faceOk && (mainW != faceW || mainH != faceH))
                || (subOk && faceOk && (subW != faceW || subH != faceH))
                || (mainOk && subOk && (mainW != subW || mainH != subH));
        boolean sizeDiff = false;
        if (mainOk && faceOk && faceBytes > 0) {
            double ratio = (double) mainBytes / (double) faceBytes;
            sizeDiff = ratio < 0.75 || ratio > 1.25;
        }
        lastSpikeDistinct = lastSpikeOk && (resDiff || sizeDiff || !faceOk);
        // If HD works at all and HAL face also works at same res, still treat as "candidate"
        // for streaming — operator confirms visually from pulled JPEGs.
        if (lastSpikeOk && !lastSpikeDistinct) {
            summary.append("heuristic=SAME_AS_FACE_OR_UNKNOWN (confirm visually); ");
        } else if (lastSpikeDistinct) {
            summary.append("heuristic=LIKELY_DISTINCT; ");
        }
        lastSpikeSummary = summary.toString().trim();
        log("HD spike done: " + lastSpikeSummary);
        // Allow forehead RTMP if any HD channel opened successfully (Phase 1 wiring).
        // Distinctness is reported for docs; streaming still useful for visual compare.
        setHealthy(lastSpikeOk);
    }

    private boolean captureHdStill(int channel, File outFile) {
        // Prefer SOFTWARE_DECODE_NV21 — sample HARDWARE path needs a Surface + MediaCodec;
        // getVideoImage alone stays null without that. Listener must be set *before* openStream.
        if (captureHdStillWithDecode(channel, outFile, StreamOption.SOFTWARE_DECODE_NV21)) {
            return true;
        }
        log("NV21 path failed for ch=" + channel + "; trying HARDWARE_DECODE + getVideoImage");
        return captureHdStillWithDecode(channel, outFile, StreamOption.HARDWARE_DECODE);
    }

    private boolean captureHdStillWithDecode(int channel, File outFile, int decodeType) {
        int handle = -1;
        final CountDownLatch latch = new CountDownLatch(1);
        final AtomicReference<Bitmap> fromStream = new AtomicReference<>();
        final AtomicInteger frameCount = new AtomicInteger(0);
        final AtomicInteger lastW = new AtomicInteger(0);
        final AtomicInteger lastH = new AtomicInteger(0);
        final AtomicInteger lastLen = new AtomicInteger(0);
        MediaStreamListener listener = new MediaStreamListener() {
            @Override
            public void getVideoStream(int streamHandle, byte[] bytes, int width, int height) {
                frameCount.incrementAndGet();
                lastW.set(width);
                lastH.set(height);
                lastLen.set(bytes != null ? bytes.length : 0);
                if (bytes == null || width <= 0 || height <= 0) {
                    return;
                }
                if (fromStream.get() != null) {
                    return;
                }
                try {
                    int needNv21 = width * height * 3 / 2;
                    if (bytes.length >= needNv21) {
                        YuvImage yuv = new YuvImage(bytes, ImageFormat.NV21, width, height, null);
                        ByteArrayOutputStream baos = new ByteArrayOutputStream();
                        yuv.compressToJpeg(new Rect(0, 0, width, height), 90, baos);
                        byte[] jpeg = baos.toByteArray();
                        Bitmap bmp = android.graphics.BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
                        if (bmp != null && bmp.getWidth() > 0) {
                            fromStream.set(bmp);
                            latch.countDown();
                        }
                    } else {
                        log("stream frame ch=" + channel + " " + width + "x" + height
                                + " bytes=" + bytes.length + " (not NV21-sized; likely H.264)");
                    }
                } catch (Exception e) {
                    Log.w(TAG, "yuv decode", e);
                }
            }

            @Override
            public void getAudioStream(int i, byte[] bytes) {
            }
        };
        try {
            hdCameraManager.setMediaListener(listener);
            StreamOption opt = new StreamOption();
            opt.setChannel((byte) channel);
            opt.setDecodType(decodeType);
            opt.setJustIframe(false);
            OperationResult op = hdCameraManager.openStream(opt);
            String result = op != null ? op.getResult() : "-1";
            handle = Integer.parseInt(result.trim());
            log("openStream ch=" + channel + " decode=" + decodeType + " handle=" + handle);
            if (handle == -1) {
                return false;
            }
            // Poll getVideoImage while waiting for NV21 listener frames (sample capture path).
            Bitmap bmp = null;
            for (int i = 0; i < 40; i++) {
                Thread.sleep(150);
                bmp = fromStream.get();
                if (bmp == null) {
                    bmp = hdCameraManager.getVideoImage();
                }
                if (bmp != null && bmp.getWidth() > 0) {
                    break;
                }
            }
            if (bmp == null) {
                latch.await(2, TimeUnit.SECONDS);
                bmp = fromStream.get();
            }
            log("ch=" + channel + " decode=" + decodeType
                    + " frames=" + frameCount.get()
                    + " last=" + lastW.get() + "x" + lastH.get()
                    + " len=" + lastLen.get()
                    + " bmp=" + (bmp != null ? bmp.getWidth() + "x" + bmp.getHeight() : "null"));
            if (bmp == null || outFile == null) {
                log("no bitmap for channel " + channel + " decode=" + decodeType);
                return false;
            }
            try (FileOutputStream fos = new FileOutputStream(outFile)) {
                bmp.compress(Bitmap.CompressFormat.JPEG, 90, fos);
            }
            log("wrote " + outFile.getAbsolutePath() + " " + bmp.getWidth() + "x" + bmp.getHeight());
            return true;
        } catch (Exception e) {
            log("captureHdStill FAIL: " + e.getMessage());
            Log.e(TAG, "still", e);
            return false;
        } finally {
            if (handle != -1 && hdCameraManager != null) {
                try {
                    hdCameraManager.closeStream(handle);
                } catch (Exception ignored) {
                }
            }
        }
    }

    /** HAL face still with hard timeout — Camera.open can hang when Face RTMP holds the camera. */
    private boolean captureHalFaceStillTimed(File outFile, long timeoutMs) {
        final AtomicBoolean ok = new AtomicBoolean(false);
        final CountDownLatch done = new CountDownLatch(1);
        Thread t = new Thread(() -> {
            try {
                ok.set(captureHalFaceStill(outFile));
            } finally {
                done.countDown();
            }
        }, "hdcam-face-hal");
        t.start();
        try {
            if (!done.await(timeoutMs, TimeUnit.MILLISECONDS)) {
                log("HAL face still timed out after " + timeoutMs + "ms (Camera likely held by Face RTMP)");
                t.interrupt();
                return false;
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return false;
        }
        return ok.get();
    }

    private boolean captureHalFaceStill(File outFile) {
        Camera camera = null;
        try {
            camera = Camera.open(0);
            if (camera == null) {
                log("HAL Camera.open(0) null");
                return false;
            }
            Camera.Parameters params = camera.getParameters();
            Camera.Size best = null;
            for (Camera.Size s : params.getSupportedPreviewSizes()) {
                if (best == null || s.width * s.height > best.width * best.height) {
                    best = s;
                }
            }
            if (best != null) {
                params.setPreviewSize(best.width, best.height);
                camera.setParameters(params);
            }
            final CountDownLatch latch = new CountDownLatch(1);
            final AtomicBoolean ok = new AtomicBoolean(false);
            camera.setPreviewTexture(new android.graphics.SurfaceTexture(0));
            camera.startPreview();
            Thread.sleep(400);
            camera.takePicture(null, null, (data, cam) -> {
                try {
                    if (outFile != null && data != null) {
                        try (FileOutputStream fos = new FileOutputStream(outFile)) {
                            fos.write(data);
                        }
                        ok.set(true);
                        log("wrote HAL face " + outFile.getAbsolutePath() + " bytes=" + data.length);
                    }
                } catch (Exception e) {
                    log("HAL still write FAIL: " + e.getMessage());
                } finally {
                    latch.countDown();
                }
            });
            latch.await(5, TimeUnit.SECONDS);
            return ok.get();
        } catch (Exception e) {
            log("HAL face still FAIL: " + e.getMessage());
            return false;
        } finally {
            if (camera != null) {
                try {
                    camera.stopPreview();
                } catch (Exception ignored) {
                }
                try {
                    camera.release();
                } catch (Exception ignored) {
                }
            }
        }
    }

    private static int[] readJpegSize(File f) {
        try {
            Bitmap bmp = android.graphics.BitmapFactory.decodeFile(f.getAbsolutePath());
            if (bmp != null) {
                return new int[]{bmp.getWidth(), bmp.getHeight()};
            }
        } catch (Exception ignored) {
        }
        return new int[]{0, 0};
    }

    void start() {
        if (hdCameraManager == null) {
            log("Cannot start forehead — HDCameraManager null");
            setHealthy(false);
            return;
        }
        if (publishing.get()) {
            log("Forehead already publishing");
            return;
        }
        startRequested.set(true);
        streamThread = new Thread(this::runPublishLoop, "hd-forehead-rtmp");
        streamThread.start();
    }

    void stop() {
        startRequested.set(false);
        publishing.set(false);
        releasePublish();
        closeHdHandle();
        setHealthy(lastSpikeOk);
        log("Forehead publish stopped");
    }

    private void runPublishLoop() {
        try {
            // NV21 software decode feeds RTMP without needing a SurfaceView (sample HW path does).
            final AtomicReference<Bitmap> latest = new AtomicReference<>();
            hdCameraManager.setMediaListener(new MediaStreamListener() {
                @Override
                public void getVideoStream(int handle, byte[] bytes, int width, int height) {
                    if (bytes == null || width <= 0 || height <= 0) return;
                    int need = width * height * 3 / 2;
                    if (bytes.length < need) return;
                    try {
                        YuvImage yuv = new YuvImage(bytes, ImageFormat.NV21, width, height, null);
                        ByteArrayOutputStream baos = new ByteArrayOutputStream();
                        yuv.compressToJpeg(new Rect(0, 0, width, height), 80, baos);
                        byte[] jpeg = baos.toByteArray();
                        Bitmap bmp = android.graphics.BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
                        if (bmp != null) {
                            Bitmap prev = latest.getAndSet(bmp);
                            if (prev != null && prev != bmp) {
                                prev.recycle();
                            }
                        }
                    } catch (Exception e) {
                        Log.w(TAG, "nv21 frame", e);
                    }
                }

                @Override
                public void getAudioStream(int i, byte[] bytes) {
                }
            });
            StreamOption opt = new StreamOption();
            opt.setChannel((byte) StreamOption.MAIN_STREAM);
            opt.setDecodType(StreamOption.SOFTWARE_DECODE_NV21);
            opt.setJustIframe(false);
            OperationResult op = hdCameraManager.openStream(opt);
            streamHandle = Integer.parseInt(op.getResult().trim());
            if (streamHandle == -1) {
                opt.setChannel((byte) StreamOption.SUB_STREAM);
                op = hdCameraManager.openStream(opt);
                streamHandle = Integer.parseInt(op.getResult().trim());
            }
            if (streamHandle == -1) {
                log("FAIL forehead openStream");
                setHealthy(false);
                return;
            }
            bufferSource = new BufferVideoSource(BufferVideoSource.Format.ARGB, STREAM_BITRATE);
            rtmpStream = new GenericStream(appContext, this, bufferSource, new NoAudioSource());
            if (!rtmpStream.prepareVideo(STREAM_W, STREAM_H, STREAM_BITRATE, 10)) {
                log("FAIL forehead prepareVideo");
                setHealthy(false);
                return;
            }
            try {
                rtmpStream.prepareAudio(44100, false, 64 * 1024);
            } catch (Exception ignored) {
            }
            String url = "rtmp://" + brokerIp + ":1935/live/forehead";
            rtmpStream.startStream(url);
            publishing.set(true);
            setHealthy(true);
            log("Forehead RTMP → " + url);

            final int[] scaled = new int[STREAM_W * STREAM_H];
            while (publishing.get() && startRequested.get()) {
                Bitmap bmp = latest.get();
                if (bmp == null) {
                    bmp = hdCameraManager.getVideoImage();
                }
                if (bmp != null && bmp.getWidth() > 0) {
                    Bitmap scaledBmp = Bitmap.createScaledBitmap(bmp, STREAM_W, STREAM_H, true);
                    scaledBmp.getPixels(scaled, 0, STREAM_W, 0, 0, STREAM_W, STREAM_H);
                    bufferSource.setBuffer(scaled);
                    if (scaledBmp != bmp) {
                        scaledBmp.recycle();
                    }
                }
                Thread.sleep(100);
            }
        } catch (Exception e) {
            log("Forehead stream FAIL: " + e.getMessage());
            Log.e(TAG, "publish", e);
            setHealthy(false);
        } finally {
            publishing.set(false);
            releasePublish();
            closeHdHandle();
        }
    }

    private void closeHdHandle() {
        if (streamHandle != -1 && hdCameraManager != null) {
            try {
                hdCameraManager.closeStream(streamHandle);
            } catch (Exception ignored) {
            }
            streamHandle = -1;
        }
    }

    private void releasePublish() {
        GenericStream stream = rtmpStream;
        rtmpStream = null;
        bufferSource = null;
        if (stream == null) return;
        try {
            if (stream.isStreaming()) {
                stream.stopStream();
            }
            stream.release();
        } catch (Exception e) {
            Log.w(TAG, "rtmp stop", e);
        }
    }

    private void setHealthy(boolean ok) {
        boolean prev = healthy.getAndSet(ok);
        if (prev != ok && listener != null) {
            mainHandler.post(() -> listener.onForeheadAvailabilityChanged(ok));
        }
    }

    private void log(String line) {
        Log.i(TAG, line);
        if (listener != null) {
            mainHandler.post(() -> listener.onHdLog(line));
        }
    }

    @Override
    public void onConnectionStarted(String url) {
        log("Forehead RTMP connecting " + url);
    }

    @Override
    public void onConnectionSuccess() {
        log("Forehead RTMP connected");
        setHealthy(true);
    }

    @Override
    public void onConnectionFailed(String reason) {
        log("Forehead RTMP failed: " + reason);
    }

    @Override
    public void onNewBitrate(long bitrate) {
    }

    @Override
    public void onDisconnect() {
        log("Forehead RTMP disconnected");
    }

    @Override
    public void onAuthError() {
        log("Forehead RTMP auth error");
    }

    @Override
    public void onAuthSuccess() {
    }
}
