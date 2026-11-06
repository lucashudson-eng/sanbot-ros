package com.qihan.HumanFollow;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.PointF;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.media.FaceDetector;
import android.media.Image;
import android.media.ImageReader;
import android.media.MediaCodec;
import android.media.MediaFormat;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.view.Surface;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * On-device person finder (Nano has no {@code com.hfisone} / Orbbec body SDK hook).
 * Combines {@link FaceDetector} (expanded to a body box) with motion-blob person detect.
 */
final class LocalPersonScanner {

    interface Callback {
        void onTargets(List<TrackTarget> targets);
    }

    private static final String TAG = "HumanFollow-person";
    private static final long MIN_SCAN_INTERVAL_MS = 220L;
    private static final int MAX_FACES = 4;
    /** FaceDetector is weak above ~VGA; try a couple of downscales. */
    private static final int[] FACE_SCAN_WIDTHS = {480, 320, 640};
    private static final float FACE_CONFIDENCE_MIN = 0.20f;
    private static final int MOTION_W = 160;
    private static final int MOTION_H = 90;
    private static final long SCAN_LOG_INTERVAL_MS = 2000L;

    private final HandlerThread thread = new HandlerThread("local-person");
    private Handler worker;
    private final Handler mainHandler;
    private final Callback callback;

    private volatile boolean running;
    private volatile boolean sdkFacesActive;
    /** When the robot is translating, frame-diff lights up walls — skip motion blobs. */
    private volatile boolean suppressMotion;
    private long lastScanAt;
    private long lastScanLogAt;
    private int decodeOk;
    private int decodeFail;
    private int facesFound;
    private int pendingFeeds;

    private MediaCodec softDecoder;
    private ImageReader imageReader;
    private int decoderW;
    private int decoderH;
    private ByteBuffer[] inputBuffers;

    private int[] prevGray;
    private int prevGrayW;
    private int prevGrayH;

    LocalPersonScanner(Handler mainHandler, Callback callback) {
        this.mainHandler = mainHandler;
        this.callback = callback;
    }

    void markSdkFacesActive() {
        sdkFacesActive = true;
    }

    void setSuppressMotion(boolean suppress) {
        // Keep prevGray so raised-hand ROI can still fire while translating.
        suppressMotion = suppress;
    }

    void start() {
        if (running) {
            return;
        }
        running = true;
        if (!thread.isAlive()) {
            thread.start();
            worker = new Handler(thread.getLooper());
        }
    }

    void stop() {
        running = false;
        if (worker != null) {
            worker.removeCallbacksAndMessages(null);
            worker.post(this::releaseDecoder);
        } else {
            releaseDecoder();
        }
        prevGray = null;
    }

    void destroy() {
        stop();
        try {
            thread.quitSafely();
        } catch (Exception ignored) {
        }
    }

    void onVideoFrame(byte[] bytes, int width, int height) {
        if (!running || bytes == null || bytes.length == 0 || worker == null) {
            return;
        }
        if (sdkFacesActive) {
            // SDK face path is live this tick — skip local decode this frame only.
            sdkFacesActive = false;
            return;
        }
        // Feed every NAL (SPS/PPS/IDR). Dropping frames unsyncs the AVC decoder.
        if (pendingFeeds > 12) {
            return;
        }
        final byte[] copy = Arrays.copyOf(bytes, bytes.length);
        final int w = width;
        final int h = height;
        pendingFeeds++;
        worker.post(() -> {
            pendingFeeds = Math.max(0, pendingFeeds - 1);
            feedAndMaybeScan(copy, w, h);
        });
    }

    private void feedAndMaybeScan(byte[] sample, int w, int h) {
        if (!running || sample == null || w <= 0 || h <= 0) {
            return;
        }
        try {
            Bitmap bmp = decodeSample(sample, w, h);
            if (bmp == null) {
                decodeFail++;
                maybeLogScanStats(0, false);
                return;
            }
            decodeOk++;
            long now = System.currentTimeMillis();
            if (now - lastScanAt < MIN_SCAN_INTERVAL_MS) {
                bmp.recycle();
                return;
            }
            lastScanAt = now;
            List<TrackTarget> targets = new ArrayList<>();
            List<TrackTarget> faces = findFaces(bmp);
            facesFound += faces.size();
            for (TrackTarget face : faces) {
                targets.add(face);
                targets.add(TrackTarget.bodyFromFace(face));
            }
            TrackTarget hand = findRaisedHand(bmp, faces);
            if (hand != null) {
                targets.add(hand);
            }
            boolean motion = false;
            if (!suppressMotion && faces.isEmpty()) {
                // Motion is a face-search hint only — never used when a face is already found.
                TrackTarget motionTarget = findMotionPerson(bmp);
                if (motionTarget != null && !overlapsAnyPerson(targets, motionTarget)) {
                    targets.add(motionTarget);
                    motion = true;
                }
            }
            bmp.recycle();
            maybeLogScanStats(faces.size(), motion);
            final List<TrackTarget> copy = targets;
            mainHandler.post(() -> callback.onTargets(copy));
        } catch (Exception e) {
            Log.w(TAG, "feedAndMaybeScan", e);
        }
    }

    private void maybeLogScanStats(int facesThisScan, boolean motion) {
        long now = System.currentTimeMillis();
        if (now - lastScanLogAt < SCAN_LOG_INTERVAL_MS) {
            return;
        }
        lastScanLogAt = now;
        Log.i(TAG, "scan faces=" + facesThisScan
                + " motion=" + motion
                + " decodeOk=" + decodeOk
                + " decodeFail=" + decodeFail
                + " facesTotal=" + facesFound
                + " suppressMotion=" + suppressMotion);
        decodeOk = 0;
        decodeFail = 0;
        facesFound = 0;
    }

    private static boolean overlapsAnyPerson(List<TrackTarget> targets, TrackTarget candidate) {
        for (TrackTarget t : targets) {
            if (t.kind != TrackTarget.Kind.PERSON) {
                continue;
            }
            double ix1 = Math.max(t.left, candidate.left);
            double iy1 = Math.max(t.top, candidate.top);
            double ix2 = Math.min(t.right, candidate.right);
            double iy2 = Math.min(t.bottom, candidate.bottom);
            if (ix2 > ix1 && iy2 > iy1) {
                double inter = (ix2 - ix1) * (iy2 - iy1);
                if (inter > 0.25 * Math.min(t.area(), candidate.area())) {
                    return true;
                }
            }
        }
        return false;
    }

    private List<TrackTarget> findFaces(Bitmap rgb) {
        List<TrackTarget> out = new ArrayList<>();
        for (int targetW : FACE_SCAN_WIDTHS) {
            addFacesAtWidth(rgb, targetW, out);
            if (!out.isEmpty()) {
                break;
            }
        }
        return out;
    }

    private void addFacesAtWidth(Bitmap rgb, int targetW, List<TrackTarget> out) {
        int srcW = rgb.getWidth();
        int srcH = rgb.getHeight();
        int bw = Math.min(targetW, srcW);
        if ((bw & 1) == 1) {
            bw -= 1;
        }
        if (bw < 64) {
            return;
        }
        int bh = Math.max(2, (int) (srcH * (bw / (float) srcW)));
        if ((bh & 1) == 1) {
            bh -= 1;
        }
        Bitmap scaled = null;
        Bitmap work = null;
        try {
            scaled = Bitmap.createScaledBitmap(rgb, bw, bh, true);
            work = scaled.getConfig() == Bitmap.Config.RGB_565
                    ? scaled.copy(Bitmap.Config.RGB_565, true)
                    : scaled.copy(Bitmap.Config.RGB_565, true);
            if (work != scaled) {
                scaled.recycle();
                scaled = null;
            }
            boostContrastRgb565(work);
            FaceDetector detector = new FaceDetector(work.getWidth(), work.getHeight(), MAX_FACES);
            FaceDetector.Face[] found = new FaceDetector.Face[MAX_FACES];
            int n = detector.findFaces(work, found);
            float fw = work.getWidth();
            float fh = work.getHeight();
            for (int i = 0; i < n; i++) {
                FaceDetector.Face face = found[i];
                if (face == null || face.confidence() < FACE_CONFIDENCE_MIN) {
                    continue;
                }
                PointF mid = new PointF();
                face.getMidPoint(mid);
                float eyes = Math.max(6f, face.eyesDistance());
                float boxW = eyes * 2.8f;
                float boxH = eyes * 3.4f;
                float left = Math.max(0, mid.x - boxW / 2f);
                float top = Math.max(0, mid.y - boxH * 0.55f);
                float right = Math.min(fw, left + boxW);
                float bottom = Math.min(fh, top + boxH);
                out.add(new TrackTarget(
                        TrackTarget.Kind.FACE,
                        left / fw, top / fh, right / fw, bottom / fh,
                        "FACE"));
            }
        } finally {
            if (work != null) {
                work.recycle();
            }
            if (scaled != null && scaled != work) {
                scaled.recycle();
            }
        }
    }

    /** Mild contrast stretch — FaceDetector is sensitive to flat indoor lighting. */
    private static void boostContrastRgb565(Bitmap rgb565) {
        if (rgb565 == null || rgb565.getConfig() != Bitmap.Config.RGB_565) {
            return;
        }
        int w = rgb565.getWidth();
        int h = rgb565.getHeight();
        int[] pixels = new int[w * h];
        rgb565.getPixels(pixels, 0, w, 0, 0, w, h);
        int min = 255;
        int max = 0;
        for (int p : pixels) {
            int y = (((p >> 16) & 0xff) * 30 + ((p >> 8) & 0xff) * 59 + (p & 0xff) * 11) / 100;
            if (y < min) min = y;
            if (y > max) max = y;
        }
        int span = Math.max(32, max - min);
        if (span >= 200) {
            return;
        }
        for (int i = 0; i < pixels.length; i++) {
            int p = pixels[i];
            int r = (p >> 16) & 0xff;
            int g = (p >> 8) & 0xff;
            int b = p & 0xff;
            r = Math.min(255, Math.max(0, (r - min) * 255 / span));
            g = Math.min(255, Math.max(0, (g - min) * 255 / span));
            b = Math.min(255, Math.max(0, (b - min) * 255 / span));
            pixels[i] = 0xff000000 | (r << 16) | (g << 8) | b;
        }
        rgb565.setPixels(pixels, 0, w, 0, 0, w, h);
    }

    /** Update prevGray frame; returns gray buffer (caller must not mutate after next call). */
    private int[] updateGray(Bitmap rgb) {
        Bitmap small = Bitmap.createScaledBitmap(rgb, MOTION_W, MOTION_H, true);
        int[] gray = new int[MOTION_W * MOTION_H];
        int[] pixels = new int[MOTION_W * MOTION_H];
        small.getPixels(pixels, 0, MOTION_W, 0, 0, MOTION_W, MOTION_H);
        if (small != rgb) {
            small.recycle();
        }
        for (int i = 0; i < pixels.length; i++) {
            int p = pixels[i];
            int r = (p >> 16) & 0xff;
            int g = (p >> 8) & 0xff;
            int b = p & 0xff;
            gray[i] = (r * 30 + g * 59 + b * 11) / 100;
        }
        return gray;
    }

    /**
     * Accessibility: raised hand / open palm above a detected face.
     * Uses frame-diff in a ROI above the face (wave or hold hand up).
     */
    private TrackTarget findRaisedHand(Bitmap rgb, List<TrackTarget> faces) {
        if (faces == null || faces.isEmpty()) {
            return null;
        }
        int[] gray = updateGray(rgb);
        if (prevGray == null || prevGrayW != MOTION_W || prevGrayH != MOTION_H) {
            prevGray = gray;
            prevGrayW = MOTION_W;
            prevGrayH = MOTION_H;
            return null;
        }

        TrackTarget bestFace = faces.get(0);
        for (TrackTarget f : faces) {
            if (f.area() > bestFace.area()) {
                bestFace = f;
            }
        }

        int x0 = (int) Math.max(0, (bestFace.centerX() - 0.22) * MOTION_W);
        int x1 = (int) Math.min(MOTION_W - 1, (bestFace.centerX() + 0.22) * MOTION_W);
        int y1 = (int) Math.max(1, bestFace.top * MOTION_H);
        int y0 = (int) Math.max(0, y1 - MOTION_H * 0.28);
        if (y1 - y0 < 6 || x1 - x0 < 6) {
            prevGray = gray;
            return null;
        }

        int minX = MOTION_W;
        int minY = MOTION_H;
        int maxX = 0;
        int maxY = 0;
        int hits = 0;
        for (int y = y0; y < y1; y++) {
            int row = y * MOTION_W;
            for (int x = x0; x <= x1; x++) {
                int diff = Math.abs(gray[row + x] - prevGray[row + x]);
                if (diff > 32) {
                    hits++;
                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                }
            }
        }
        prevGray = gray;

        // Sustained motion above the face — hand wave / raised palm
        if (hits < 45) {
            return null;
        }
        float bw = (maxX - minX + 1f) / MOTION_W;
        float bh = (maxY - minY + 1f) / MOTION_H;
        if (bw < 0.04f || bh < 0.03f || bw > 0.45f) {
            return null;
        }
        double left = minX / (double) MOTION_W;
        double right = (maxX + 1) / (double) MOTION_W;
        double top = minY / (double) MOTION_H;
        double bottom = (maxY + 1) / (double) MOTION_H;
        return new TrackTarget(TrackTarget.Kind.FACE, left, top, right, bottom, "HAND_UP");
    }

    /**
     * Frame-diff motion blob → approximate standing person box.
     * Ignores the bottom 15% (floor / wheels) and tiny flicker.
     */
    private TrackTarget findMotionPerson(Bitmap rgb) {
        int[] gray = updateGray(rgb);

        if (prevGray == null || prevGrayW != MOTION_W || prevGrayH != MOTION_H) {
            prevGray = gray;
            prevGrayW = MOTION_W;
            prevGrayH = MOTION_H;
            return null;
        }

        int yMax = (int) (MOTION_H * 0.88f);
        int minX = MOTION_W;
        int minY = MOTION_H;
        int maxX = 0;
        int maxY = 0;
        int hits = 0;
        for (int y = 0; y < yMax; y++) {
            int row = y * MOTION_W;
            for (int x = 0; x < MOTION_W; x++) {
                int diff = Math.abs(gray[row + x] - prevGray[row + x]);
                if (diff > 28) {
                    hits++;
                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                }
            }
        }
        prevGray = gray;

        if (hits < 90) {
            return null;
        }
        float bw = (maxX - minX + 1f) / MOTION_W;
        float bh = (maxY - minY + 1f) / MOTION_H;
        // Person-ish upright blob; reject wall-sized / full-frame ego-motion flashes
        if (bh < 0.18f || bw < 0.08f || bw > 0.60f || bh > 0.80f) {
            return null;
        }
        if (bh < bw * 0.95f) {
            // Prefer upright (taller than wide)
            return null;
        }
        // Pad box a bit
        double left = Math.max(0, (minX / (double) MOTION_W) - 0.04);
        double right = Math.min(1, (maxX / (double) MOTION_W) + 0.04);
        double top = Math.max(0, (minY / (double) MOTION_H) - 0.04);
        double bottom = Math.min(1, (maxY / (double) MOTION_H) + 0.06);
        // Motion boxes are huge vs face height — pass a scaled range so follow
        // does not treat every blob as “too close” and reverse forever.
        double boxH = Math.max(0.01, bottom - top);
        return new TrackTarget(TrackTarget.Kind.PERSON, left, top, right, bottom, "MOTION",
                Math.min(0.18, boxH * 0.22));
    }

    private Bitmap decodeSample(byte[] sample, int width, int height) {
        ensureDecoder(width, height);
        if (softDecoder == null || imageReader == null) {
            return null;
        }
        try {
            int inIndex = softDecoder.dequeueInputBuffer(2000);
            if (inIndex < 0) {
                return null;
            }
            ByteBuffer inBuf = inputBuffers[inIndex];
            inBuf.clear();
            inBuf.put(sample);
            softDecoder.queueInputBuffer(inIndex, 0, sample.length, System.nanoTime() / 1000, 0);

            MediaCodec.BufferInfo info = new MediaCodec.BufferInfo();
            // Drain quickly; keep feeding subsequent frames even when no image yet.
            for (int i = 0; i < 8; i++) {
                int outIndex = softDecoder.dequeueOutputBuffer(info, 2000);
                if (outIndex == MediaCodec.INFO_TRY_AGAIN_LATER) {
                    break;
                }
                if (outIndex == MediaCodec.INFO_OUTPUT_BUFFERS_CHANGED
                        || outIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                    continue;
                }
                if (outIndex >= 0) {
                    softDecoder.releaseOutputBuffer(outIndex, true);
                    Image image = imageReader.acquireLatestImage();
                    if (image == null) {
                        return null;
                    }
                    try {
                        return imageToRgb565(image);
                    } finally {
                        image.close();
                    }
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "decodeSample", e);
            releaseDecoder();
        }
        return null;
    }

    private void ensureDecoder(int width, int height) {
        if (softDecoder != null && decoderW == width && decoderH == height) {
            return;
        }
        releaseDecoder();
        try {
            imageReader = ImageReader.newInstance(width, height, ImageFormat.YUV_420_888, 2);
            Surface surface = imageReader.getSurface();
            MediaFormat format = MediaFormat.createVideoFormat("video/avc", width, height);
            // Nano stream needs the Allwinner HW AVC decoder (Google SW fails
            // on these NAL units). Preview uses a second instance of the same.
            softDecoder = MediaCodec.createDecoderByType("video/avc");
            softDecoder.configure(format, surface, null, 0);
            softDecoder.start();
            inputBuffers = softDecoder.getInputBuffers();
            decoderW = width;
            decoderH = height;
            Log.i(TAG, "soft decoder ready " + width + "x" + height
                    + " codec=" + softDecoder.getName());
        } catch (Exception e) {
            Log.e(TAG, "ensureDecoder", e);
            releaseDecoder();
        }
    }

    private void releaseDecoder() {
        try {
            if (softDecoder != null) {
                softDecoder.stop();
                softDecoder.release();
            }
        } catch (Exception ignored) {
        }
        softDecoder = null;
        inputBuffers = null;
        try {
            if (imageReader != null) {
                imageReader.close();
            }
        } catch (Exception ignored) {
        }
        imageReader = null;
        decoderW = 0;
        decoderH = 0;
    }

    private static Bitmap imageToRgb565(Image image) {
        int width = image.getWidth();
        int height = image.getHeight();
        if ((width & 1) == 1) {
            width -= 1;
        }
        byte[] nv21 = yuv420ToNv21(image);
        YuvImage yuv = new YuvImage(nv21, ImageFormat.NV21, image.getWidth(), height, null);
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuv.compressToJpeg(new Rect(0, 0, width, height), 90, out);
        byte[] jpeg = out.toByteArray();
        Bitmap full = android.graphics.BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
        if (full == null) {
            return null;
        }
        Bitmap rgb = full.copy(Bitmap.Config.RGB_565, false);
        if (rgb != full) {
            full.recycle();
        }
        if (rgb.getWidth() != width) {
            Bitmap cropped = Bitmap.createBitmap(rgb, 0, 0, width, rgb.getHeight());
            rgb.recycle();
            return cropped;
        }
        return rgb;
    }

    private static byte[] yuv420ToNv21(Image image) {
        int width = image.getWidth();
        int height = image.getHeight();
        Image.Plane[] planes = image.getPlanes();
        ByteBuffer yBuf = planes[0].getBuffer();
        ByteBuffer uBuf = planes[1].getBuffer();
        ByteBuffer vBuf = planes[2].getBuffer();
        int yRowStride = planes[0].getRowStride();
        int uvRowStride = planes[1].getRowStride();
        int uvPixelStride = planes[1].getPixelStride();

        byte[] nv21 = new byte[width * height * 3 / 2];
        int pos = 0;
        for (int row = 0; row < height; row++) {
            yBuf.position(row * yRowStride);
            yBuf.get(nv21, pos, width);
            pos += width;
        }
        int uvHeight = height / 2;
        int uvWidth = width / 2;
        int uvPos = width * height;
        for (int row = 0; row < uvHeight; row++) {
            for (int col = 0; col < uvWidth; col++) {
                int uIndex = row * uvRowStride + col * uvPixelStride;
                int vIndex = row * planes[2].getRowStride() + col * planes[2].getPixelStride();
                nv21[uvPos++] = vBuf.get(vIndex);
                nv21[uvPos++] = uBuf.get(uIndex);
            }
        }
        return nv21;
    }
}
