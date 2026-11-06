package com.grin.sanbotrosbridge;

import android.content.Context;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.SurfaceTexture;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import com.pedro.common.ConnectChecker;
import com.pedro.encoder.input.sources.audio.NoAudioSource;
import com.pedro.encoder.input.sources.video.BufferVideoSource;
import com.pedro.library.generic.GenericStream;

import java.io.ByteArrayOutputStream;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * External UVC webcam into the Nano (USB host) → RTMP {@code live/usb}.
 * Uses Android Camera HAL cameraId &gt; 0 when a non-Orbbec USB camera enumerates.
 * Mutually exclusive with Orbbec on limited host ports — caller should stop body first.
 */
final class UvcUsbStreamer implements ConnectChecker {
    interface Listener {
        void onUsbCamLog(String message);

        void onUsbCameraAvailabilityChanged(boolean available);
    }

    private static final String TAG = "UvcUsb";
    private static final int VENDOR_ORBBEC = 0x2bc5;
    private static final int STREAM_W = 640;
    private static final int STREAM_H = 480;
    private static final int STREAM_BITRATE = 800_000;

    private final Context appContext;
    private final Listener listener;
    private final UsbManager usbManager;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final AtomicBoolean publishing = new AtomicBoolean(false);
    private final AtomicBoolean healthy = new AtomicBoolean(false);
    private final AtomicBoolean startRequested = new AtomicBoolean(false);

    private GenericStream rtmpStream;
    private BufferVideoSource bufferSource;
    private Thread streamThread;
    private Camera camera;
    private String brokerIp = "192.168.1.4";
    private int cameraId = -1;

    UvcUsbStreamer(Context context, Listener listener) {
        this.appContext = context.getApplicationContext();
        this.listener = listener;
        this.usbManager = (UsbManager) appContext.getSystemService(Context.USB_SERVICE);
    }

    void setBrokerIp(String ip) {
        if (ip != null && !ip.trim().isEmpty()) {
            brokerIp = ip.trim();
        }
    }

    boolean isHealthy() {
        return healthy.get();
    }

    /** Non-Orbbec USB devices currently attached (host). */
    int countExternalUsbCandidates() {
        if (usbManager == null) return 0;
        int n = 0;
        HashMap<String, UsbDevice> map = usbManager.getDeviceList();
        for (UsbDevice d : map.values()) {
            if (d.getVendorId() != VENDOR_ORBBEC) {
                n++;
            }
        }
        return n;
    }

    int findSecondaryCameraId() {
        int count = Camera.getNumberOfCameras();
        // Prefer id >= 1; Camera 0 is sunxi head on Nano.
        for (int id = 1; id < count; id++) {
            return id;
        }
        return count > 1 ? 1 : -1;
    }

    void refreshAvailability() {
        int camId = findSecondaryCameraId();
        int usbN = countExternalUsbCandidates();
        boolean ok = camId >= 0 || usbN > 0;
        log("UVC probe cameras=" + Camera.getNumberOfCameras()
                + " secondaryId=" + camId + " nonOrbbecUsb=" + usbN);
        if (!publishing.get()) {
            setHealthy(ok && camId >= 0);
        }
    }

    void start() {
        cameraId = findSecondaryCameraId();
        if (cameraId < 0) {
            log("No secondary Camera HAL id (plug UVC webcam; stop Orbbec if port conflict)");
            setHealthy(false);
            return;
        }
        if (publishing.get()) {
            log("USB cam already publishing");
            return;
        }
        startRequested.set(true);
        streamThread = new Thread(this::runPublishLoop, "uvc-usb-rtmp");
        streamThread.start();
    }

    void stop() {
        startRequested.set(false);
        publishing.set(false);
        releaseCamera();
        releasePublish();
        setHealthy(findSecondaryCameraId() >= 0);
        log("USB cam publish stopped");
    }

    private void runPublishLoop() {
        try {
            camera = Camera.open(cameraId);
            Camera.Parameters params = camera.getParameters();
            Camera.Size best = null;
            for (Camera.Size s : params.getSupportedPreviewSizes()) {
                if (s.width == STREAM_W && s.height == STREAM_H) {
                    best = s;
                    break;
                }
                if (best == null || s.width * s.height > best.width * best.height) {
                    best = s;
                }
            }
            final int previewW = best != null ? best.width : STREAM_W;
            final int previewH = best != null ? best.height : STREAM_H;
            if (best != null) {
                params.setPreviewSize(previewW, previewH);
                params.setPreviewFormat(ImageFormat.NV21);
                camera.setParameters(params);
            }

            bufferSource = new BufferVideoSource(BufferVideoSource.Format.ARGB, STREAM_BITRATE);
            rtmpStream = new GenericStream(appContext, this, bufferSource, new NoAudioSource());
            if (!rtmpStream.prepareVideo(STREAM_W, STREAM_H, STREAM_BITRATE, 15)) {
                log("FAIL usb prepareVideo");
                setHealthy(false);
                return;
            }
            try {
                rtmpStream.prepareAudio(44100, false, 64 * 1024);
            } catch (Exception ignored) {
            }
            String url = "rtmp://" + brokerIp + ":1935/live/usb";
            rtmpStream.startStream(url);
            publishing.set(true);
            setHealthy(true);
            log("USB RTMP → " + url + " camId=" + cameraId + " preview=" + previewW + "x" + previewH);

            final int[] scaled = new int[STREAM_W * STREAM_H];
            final AtomicBoolean frameGate = new AtomicBoolean(false);
            camera.setPreviewCallback((data, cam) -> {
                if (!publishing.get() || data == null) return;
                if (!frameGate.compareAndSet(false, true)) return;
                try {
                    YuvImage yuv = new YuvImage(data, ImageFormat.NV21, previewW, previewH, null);
                    ByteArrayOutputStream baos = new ByteArrayOutputStream();
                    yuv.compressToJpeg(new Rect(0, 0, previewW, previewH), 80, baos);
                    byte[] jpeg = baos.toByteArray();
                    android.graphics.Bitmap bmp =
                            android.graphics.BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
                    if (bmp != null) {
                        android.graphics.Bitmap scaledBmp =
                                android.graphics.Bitmap.createScaledBitmap(bmp, STREAM_W, STREAM_H, true);
                        scaledBmp.getPixels(scaled, 0, STREAM_W, 0, 0, STREAM_W, STREAM_H);
                        bufferSource.setBuffer(scaled);
                        if (scaledBmp != bmp) scaledBmp.recycle();
                        bmp.recycle();
                    }
                } catch (Exception e) {
                    Log.w(TAG, "frame", e);
                } finally {
                    frameGate.set(false);
                }
            });
            camera.setPreviewTexture(new SurfaceTexture(10));
            camera.startPreview();

            while (publishing.get() && startRequested.get()) {
                Thread.sleep(200);
            }
        } catch (Exception e) {
            log("USB stream FAIL: " + e.getMessage());
            Log.e(TAG, "publish", e);
            setHealthy(false);
        } finally {
            publishing.set(false);
            releaseCamera();
            releasePublish();
        }
    }

    private void releaseCamera() {
        if (camera != null) {
            try {
                camera.setPreviewCallback(null);
                camera.stopPreview();
            } catch (Exception ignored) {
            }
            try {
                camera.release();
            } catch (Exception ignored) {
            }
            camera = null;
        }
    }

    private void releasePublish() {
        GenericStream stream = rtmpStream;
        rtmpStream = null;
        bufferSource = null;
        if (stream == null) return;
        try {
            if (stream.isStreaming()) stream.stopStream();
            stream.release();
        } catch (Exception e) {
            Log.w(TAG, "rtmp stop", e);
        }
    }

    private void setHealthy(boolean ok) {
        boolean prev = healthy.getAndSet(ok);
        if (prev != ok && listener != null) {
            mainHandler.post(() -> listener.onUsbCameraAvailabilityChanged(ok));
        }
    }

    private void log(String line) {
        Log.i(TAG, line);
        if (listener != null) {
            mainHandler.post(() -> listener.onUsbCamLog(line));
        }
    }

    @Override
    public void onConnectionStarted(String url) {
        log("USB RTMP connecting " + url);
    }

    @Override
    public void onConnectionSuccess() {
        log("USB RTMP connected");
        setHealthy(true);
    }

    @Override
    public void onConnectionFailed(String reason) {
        log("USB RTMP failed: " + reason);
    }

    @Override
    public void onNewBitrate(long bitrate) {
    }

    @Override
    public void onDisconnect() {
        log("USB RTMP disconnected");
    }

    @Override
    public void onAuthError() {
        log("USB RTMP auth error");
    }

    @Override
    public void onAuthSuccess() {
    }
}
