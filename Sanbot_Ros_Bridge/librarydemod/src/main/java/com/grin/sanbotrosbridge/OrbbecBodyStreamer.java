package com.grin.sanbotrosbridge;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Build;
import android.util.Log;

import com.pedro.common.ConnectChecker;
import com.pedro.encoder.input.sources.audio.NoAudioSource;
import com.pedro.encoder.input.sources.video.BufferVideoSource;
import com.pedro.library.generic.GenericStream;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Orbbec Astra → OpenNI ARGB → Pedro BufferVideoSource → rtmp://broker:1935/live/body.
 * Ported from orbbec-probe 0.2.0-m1.
 * <p>
 * OpenNI on Nano does not safely re-init after xnContextRelease — keep the USB/OpenNI
 * session for the process lifetime and only start/stop the RTMP publish loop.
 */
final class OrbbecBodyStreamer implements ConnectChecker {
    interface Listener {
        void onBodyLog(String message);

        void onBodyAvailabilityChanged(boolean available);

        /** Depth RTMP availability ({@code live/depth}). Default no-op via MainActivity. */
        default void onDepthAvailabilityChanged(boolean available) {}
    }

    private static final String TAG = "OrbbecBody";
    private static final String ACTION_USB = "com.grin.sanbotrosbridge.ORBBEC_USB_PERMISSION";
    private static final int VENDOR_ORBBEC = 0x2bc5;
    private static final int STREAM_W = 640;
    private static final int STREAM_H = 480;
    private static final int STREAM_BITRATE = 800_000;

    private static final String[] ORBBEC_GAME_PACKAGES = {
            "com.orbbec.WallCome",
            "com.Orbbec.BraveJelly",
            "com.Orbbec.Wukong",
            "com.Orbbec.MagicSalad",
            "com.Orbbec.SpeedAirship",
            "au.com.brien.orbbecprobe"
    };

    private final Context appContext;
    private final Listener listener;
    private final UsbManager usbManager;
    private final AtomicBoolean publishing = new AtomicBoolean(false);
    private final AtomicBoolean sessionOpen = new AtomicBoolean(false);
    private final AtomicBoolean healthy = new AtomicBoolean(false);
    private final AtomicBoolean depthHealthy = new AtomicBoolean(false);
    private final AtomicBoolean startRequested = new AtomicBoolean(false);
    private final AtomicBoolean depthRequested = new AtomicBoolean(true);
    private final AtomicBoolean opening = new AtomicBoolean(false);

    private UsbDeviceConnection usbConn;
    private GenericStream rtmpStream;
    private GenericStream depthRtmpStream;
    private BufferVideoSource bufferSource;
    private BufferVideoSource depthBufferSource;
    private Thread streamThread;
    private String brokerIp = "192.168.1.4";
    private boolean receiverRegistered = false;
    private int srcW;
    private int srcH;

    private final BroadcastReceiver usbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!ACTION_USB.equals(intent.getAction())) return;
            UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            boolean granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);
            log("USB permission granted=" + granted);
            if (granted && device != null && startRequested.get()) {
                beginStreamThread(device);
            } else if (!granted) {
                startRequested.set(false);
                opening.set(false);
                setHealthy(false);
            }
        }
    };

    OrbbecBodyStreamer(Context context, Listener listener) {
        this.appContext = context.getApplicationContext();
        this.listener = listener;
        this.usbManager = (UsbManager) appContext.getSystemService(Context.USB_SERVICE);
    }

    boolean isHealthy() {
        return healthy.get();
    }

    boolean isDepthHealthy() {
        return depthHealthy.get();
    }

    boolean isStreaming() {
        return publishing.get();
    }

    boolean hasSession() {
        return sessionOpen.get();
    }

    /** When true (default), color loop also publishes grayscale depth → live/depth. */
    void setDepthPublishEnabled(boolean enabled) {
        depthRequested.set(enabled);
        if (!enabled) {
            releaseDepthPublishResources();
            setDepthHealthy(false);
        }
    }

    void setBrokerIp(String ip) {
        if (ip != null && !ip.trim().isEmpty()) {
            brokerIp = ip.trim();
        }
    }

    void register() {
        if (receiverRegistered) return;
        IntentFilter filter = new IntentFilter(ACTION_USB);
        if (Build.VERSION.SDK_INT >= 33) {
            appContext.registerReceiver(usbReceiver, filter, Context.RECEIVER_NOT_EXPORTED);
        } else {
            appContext.registerReceiver(usbReceiver, filter);
        }
        receiverRegistered = true;
    }

    void unregister() {
        stopPublish();
        releaseSession();
        if (receiverRegistered) {
            try {
                appContext.unregisterReceiver(usbReceiver);
            } catch (Exception ignored) {
            }
            receiverRegistered = false;
        }
    }

    /** Start (or resume) body RTMP. Reuses OpenNI session if already open. */
    void start() {
        startRequested.set(true);
        if (publishing.get()) {
            log("Body stream already publishing");
            return;
        }
        if (opening.get()) {
            log("Body open already in progress");
            return;
        }
        forceStopOrbbecGames();
        UsbDevice device = findOrbbec();
        if (device == null) {
            log("No Orbbec USB device (0x2bc5) — body camera unavailable");
            startRequested.set(false);
            setHealthy(false);
            return;
        }
        if (usbManager.hasPermission(device)) {
            beginStreamThread(device);
        } else {
            log("Requesting USB permission for Astra…");
            int flags = Build.VERSION.SDK_INT >= 31 ? PendingIntent.FLAG_MUTABLE : 0;
            PendingIntent pi = PendingIntent.getBroadcast(appContext, 0, new Intent(ACTION_USB), flags);
            usbManager.requestPermission(device, pi);
        }
    }

    /** Stop RTMP publish only — keep USB/OpenNI open for a clean restart. */
    void stop() {
        stopPublish();
    }

    private void stopPublish() {
        startRequested.set(false);
        publishing.set(false);
        Thread t = streamThread;
        if (t != null) {
            try {
                t.join(3000);
            } catch (InterruptedException ignored) {
            }
        }
        streamThread = null;
        // Thread finally already released Pedro; call again only if join timed out.
        releasePublishResources();
        if (!sessionOpen.get()) {
            setHealthy(false);
        }
    }

    private void releaseSession() {
        stopPublish();
        try {
            OrbbecNative.nativeClose();
        } catch (Throwable ignored) {
        }
        closeUsbQuietly();
        sessionOpen.set(false);
        opening.set(false);
        setHealthy(false);
    }

    private void beginStreamThread(UsbDevice device) {
        if (publishing.get() || opening.get()) {
            log("Body stream already running/opening");
            return;
        }
        opening.set(true);
        streamThread = new Thread(() -> runStreamLoop(device), "orbbec-body-rtmp");
        streamThread.start();
    }

    private void runStreamLoop(UsbDevice device) {
        try {
            if (!ensureSession(device)) {
                setHealthy(false);
                return;
            }
            if (!startRequested.get()) {
                return;
            }

            bufferSource = new BufferVideoSource(BufferVideoSource.Format.ARGB, STREAM_BITRATE);
            rtmpStream = new GenericStream(appContext, this, bufferSource, new NoAudioSource());
            boolean prepared = rtmpStream.prepareVideo(STREAM_W, STREAM_H, STREAM_BITRATE, 15);
            if (!prepared) {
                log("FAIL: body prepareVideo");
                setHealthy(false);
                return;
            }
            try {
                rtmpStream.prepareAudio(44100, false, 64 * 1024);
            } catch (Exception e) {
                log("body prepareAudio note: " + e.getMessage());
            }

            String url = "rtmp://" + brokerIp + ":1935/live/body";
            rtmpStream.startStream(url);
            publishing.set(true);
            setHealthy(true);
            log("Body RTMP → " + url);

            if (depthRequested.get()) {
                try {
                    depthBufferSource = new BufferVideoSource(BufferVideoSource.Format.ARGB, STREAM_BITRATE);
                    depthRtmpStream = new GenericStream(appContext, this, depthBufferSource, new NoAudioSource());
                    if (depthRtmpStream.prepareVideo(STREAM_W, STREAM_H, STREAM_BITRATE, 10)) {
                        try {
                            depthRtmpStream.prepareAudio(44100, false, 64 * 1024);
                        } catch (Exception ignored) {
                        }
                        String depthUrl = "rtmp://" + brokerIp + ":1935/live/depth";
                        depthRtmpStream.startStream(depthUrl);
                        setDepthHealthy(true);
                        log("Depth RTMP → " + depthUrl);
                    } else {
                        log("FAIL: depth prepareVideo");
                        setDepthHealthy(false);
                    }
                } catch (Exception e) {
                    log("Depth RTMP setup note: " + e.getMessage());
                    setDepthHealthy(false);
                }
            }

            final int[] argb = new int[srcW * srcH];
            final int[] depthArgb = new int[srcW * srcH];
            final int[] scaled = new int[STREAM_W * STREAM_H];
            final int[] depthScaled = new int[STREAM_W * STREAM_H];
            while (publishing.get() && startRequested.get()) {
                int rc = OrbbecNative.nativeGrabArgb(argb);
                if (rc != 1) {
                    log("body grab rc=" + rc);
                    break;
                }
                scaleNearest(argb, srcW, srcH, scaled, STREAM_W, STREAM_H);
                bufferSource.setBuffer(scaled);

                if (depthRequested.get() && depthBufferSource != null) {
                    // Same WaitAndUpdate already done in nativeGrabArgb; pull depth map only.
                    int drc = OrbbecNative.nativeGrabDepthArgb(depthArgb);
                    if (drc == 1) {
                        scaleNearest(depthArgb, srcW, srcH, depthScaled, STREAM_W, STREAM_H);
                        depthBufferSource.setBuffer(depthScaled);
                    }
                }
            }
        } catch (UnsatisfiedLinkError e) {
            log("FAIL native load: " + e.getMessage());
            Log.e(TAG, "native", e);
            setHealthy(false);
        } catch (Throwable t) {
            log("Body stream FAIL: " + t);
            Log.e(TAG, "stream", t);
            setHealthy(false);
        } finally {
            publishing.set(false);
            opening.set(false);
            releasePublishResources();
            log("Body publish stopped (OpenNI session kept=" + sessionOpen.get() + ").");
        }
    }

    private boolean ensureSession(UsbDevice device) throws Exception {
        if (sessionOpen.get()) {
            log("Reusing OpenNI session " + srcW + "x" + srcH);
            return true;
        }
        if (!openUsbAndPermission(device)) {
            return false;
        }
        File xml = copyOpenNiAssets();
        String open = OrbbecNative.nativeOpen(xml.getAbsolutePath());
        log(open);
        if (open == null || !open.startsWith("OK")) {
            closeUsbQuietly();
            return false;
        }
        int[] size = OrbbecNative.nativeFrameSize();
        srcW = size[0];
        srcH = size[1];
        boolean color = size[2] != 0;
        log("Body frame " + srcW + "x" + srcH + " mode=" + (color ? "color" : "depth-gray"));
        sessionOpen.set(true);
        return true;
    }

    private synchronized void releasePublishResources() {
        releaseDepthPublishResources();
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

    private synchronized void releaseDepthPublishResources() {
        GenericStream stream = depthRtmpStream;
        depthRtmpStream = null;
        depthBufferSource = null;
        setDepthHealthy(false);
        if (stream == null) return;
        try {
            if (stream.isStreaming()) {
                stream.stopStream();
            }
            stream.release();
        } catch (Exception e) {
            Log.w(TAG, "depth rtmp stop", e);
        }
    }

    private void setDepthHealthy(boolean ok) {
        boolean prev = depthHealthy.getAndSet(ok);
        if (prev != ok && listener != null) {
            listener.onDepthAvailabilityChanged(ok);
        }
    }

    private void forceStopOrbbecGames() {
        for (String pkg : ORBBEC_GAME_PACKAGES) {
            try {
                Runtime.getRuntime().exec(new String[]{"am", "force-stop", pkg});
            } catch (Exception ignored) {
            }
        }
    }

    private boolean openUsbAndPermission(UsbDevice device) {
        closeUsbQuietly();
        usbConn = usbManager.openDevice(device);
        if (usbConn == null) {
            log("FAIL: UsbManager.openDevice null");
            return false;
        }
        int fd = usbConn.getFileDescriptor();
        String deviceName = device.getDeviceName();
        int busNum = 0;
        int devNum = 0;
        String usbFsRoot = "/dev/bus/usb";
        if (deviceName != null) {
            String[] parts = deviceName.split("/");
            if (parts.length >= 2) {
                try {
                    busNum = Integer.parseInt(parts[parts.length - 2]);
                    devNum = Integer.parseInt(parts[parts.length - 1]);
                } catch (NumberFormatException ignored) {
                }
                if (parts.length > 2) {
                    StringBuilder sb = new StringBuilder(parts[0]);
                    for (int i = 1; i < parts.length - 2; i++) {
                        sb.append('/').append(parts[i]);
                    }
                    if (sb.length() > 0) {
                        usbFsRoot = sb.toString();
                    }
                }
            }
        }
        log("USB fd=" + fd + " path=" + deviceName);
        int permRc = OrbbecNative.onGetPermission(
                device.getVendorId(), device.getProductId(), fd, busNum, devNum, usbFsRoot);
        log("OnGetPermission rc=" + permRc);
        return true;
    }

    private File copyOpenNiAssets() throws Exception {
        File dir = appContext.getFilesDir();
        String[] names = appContext.getAssets().list("openni");
        if (names == null) {
            throw new IllegalStateException("no assets/openni");
        }
        for (String name : names) {
            File out = new File(dir, name);
            try (InputStream in = appContext.getAssets().open("openni/" + name);
                 OutputStream os = new FileOutputStream(out)) {
                byte[] buf = new byte[8192];
                int n;
                while ((n = in.read(buf)) > 0) {
                    os.write(buf, 0, n);
                }
            }
        }
        // Match probe: relative Module path in modules.xml. Copy SensorV2 into filesDir once
        // so dlopen from cwd (xnOSSetCurrentDir) succeeds; OpenNI mangles absolute Module paths.
        String libDir = appContext.getApplicationInfo().nativeLibraryDir;
        File sensorSrc = new File(libDir, "libXnDeviceSensorV2.so");
        File sensorDst = new File(dir, "libXnDeviceSensorV2.so");
        if (sensorSrc.isFile() && (!sensorDst.isFile() || sensorDst.length() != sensorSrc.length())) {
            try (InputStream in = new FileInputStream(sensorSrc);
                 OutputStream os = new FileOutputStream(sensorDst)) {
                byte[] buf = new byte[8192];
                int n;
                while ((n = in.read(buf)) > 0) {
                    os.write(buf, 0, n);
                }
            }
            log("Copied SensorV2 → " + sensorDst.getAbsolutePath());
        }
        // Keep stock relative modules.xml from assets (do not rewrite absolute paths).
        try {
            android.system.Os.setenv("OPENNI_BASE", dir.getAbsolutePath(), true);
            log("OPENNI_BASE=" + dir.getAbsolutePath() + " nativeLib=" + libDir);
        } catch (Exception e) {
            log("setenv skipped: " + e.getMessage());
        }
        return new File(dir, "SamplesConfig.xml");
    }

    private UsbDevice findOrbbec() {
        if (usbManager == null) return null;
        HashMap<String, UsbDevice> map = usbManager.getDeviceList();
        for (UsbDevice d : map.values()) {
            if (d.getVendorId() == VENDOR_ORBBEC) {
                return d;
            }
        }
        return null;
    }

    private void closeUsbQuietly() {
        if (usbConn != null) {
            try {
                usbConn.close();
            } catch (Exception ignored) {
            }
            usbConn = null;
        }
    }

    private static void scaleNearest(int[] src, int sw, int sh, int[] dst, int dw, int dh) {
        for (int y = 0; y < dh; y++) {
            int sy = y * sh / dh;
            for (int x = 0; x < dw; x++) {
                int sx = x * sw / dw;
                dst[y * dw + x] = src[sy * sw + sx];
            }
        }
    }

    private void setHealthy(boolean ok) {
        boolean prev = healthy.getAndSet(ok);
        if (prev != ok && listener != null) {
            listener.onBodyAvailabilityChanged(ok);
        }
    }

    private void log(String line) {
        Log.i(TAG, line);
        if (listener != null) {
            listener.onBodyLog(line);
        }
    }

    @Override
    public void onConnectionStarted(String url) {
        log("Body RTMP connecting " + url);
    }

    @Override
    public void onConnectionSuccess() {
        log("Body RTMP connected");
        setHealthy(true);
    }

    @Override
    public void onConnectionFailed(String reason) {
        log("Body RTMP failed: " + reason);
        setHealthy(sessionOpen.get());
    }

    @Override
    public void onNewBitrate(long bitrate) {}

    @Override
    public void onDisconnect() {
        log("Body RTMP disconnected");
        // Session may still be fine; mark unhealthy only if publish ended unexpectedly.
        if (!publishing.get()) {
            setHealthy(sessionOpen.get());
        } else {
            setHealthy(false);
        }
    }

    @Override
    public void onAuthError() {
        log("Body RTMP auth error");
    }

    @Override
    public void onAuthSuccess() {}
}
