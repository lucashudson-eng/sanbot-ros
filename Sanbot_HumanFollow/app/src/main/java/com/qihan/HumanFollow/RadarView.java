package com.qihan.HumanFollow;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.util.SparseIntArray;
import android.view.View;

/**
 * Top-down IR / PIR / obstacle radar. Front = top of the view.
 */
public class RadarView extends View {

    private static final float MAX_RANGE_CM = 80f;
    /** IR part ids treated as forward-facing for stop logic visualization. */
    private static final int[] FRONT_PARTS = {1, 2, 3, 4, 5, 6};

    private final Paint ringPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint rayPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint blipPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint bodyPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint nosePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint wedgePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint statusPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path tmpPath = new Path();

    private final SparseIntArray irDistances = new SparseIntArray();
    private boolean pirFront;
    private boolean pirBack;
    private boolean obstacleActive;
    private boolean sensorsLive;
    private long lastIrAt;
    private String statusLine = "waiting for IR…";

    public RadarView(Context context) {
        super(context);
        init();
    }

    public RadarView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public RadarView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        setWillNotDraw(false);
        ringPaint.setStyle(Paint.Style.STROKE);
        ringPaint.setStrokeWidth(1.5f);
        ringPaint.setColor(0x335EEAD4);

        rayPaint.setStyle(Paint.Style.STROKE);
        rayPaint.setStrokeCap(Paint.Cap.ROUND);

        blipPaint.setStyle(Paint.Style.FILL);
        bodyPaint.setStyle(Paint.Style.FILL);
        nosePaint.setStyle(Paint.Style.FILL);
        nosePaint.setColor(0xFF5EEAD4);
        wedgePaint.setStyle(Paint.Style.FILL);

        labelPaint.setColor(0xFF94A3B8);
        labelPaint.setTextSize(22f);
        statusPaint.setColor(0xFF5EEAD4);
        statusPaint.setTextSize(24f);
        statusPaint.setFakeBoldText(true);
    }

    public void setIrDistance(int part, int distanceCm) {
        if (part <= 0) {
            return;
        }
        irDistances.put(part, Math.max(0, distanceCm));
        lastIrAt = System.currentTimeMillis();
        sensorsLive = true;
        updateStatusLine();
        postInvalidate();
    }

    public void setPir(boolean front, boolean back) {
        pirFront = front;
        pirBack = back;
        updateStatusLine();
        postInvalidate();
    }

    public void setObstacleActive(boolean active) {
        obstacleActive = active;
        updateStatusLine();
        postInvalidate();
    }

    public boolean isSensorsLive() {
        return sensorsLive && (System.currentTimeMillis() - lastIrAt) < 3000L;
    }

    public boolean isObstacleActive() {
        return obstacleActive;
    }

    /** True if any forward IR reading is at or under {@code stopCm}. */
    public boolean isFrontBlocked(int stopCm) {
        for (int part : FRONT_PARTS) {
            int idx = irDistances.indexOfKey(part);
            if (idx >= 0 && irDistances.valueAt(idx) > 0 && irDistances.valueAt(idx) <= stopCm) {
                return true;
            }
        }
        // Also treat any very close reading as block if we don't know part map well
        for (int i = 0; i < irDistances.size(); i++) {
            int cm = irDistances.valueAt(i);
            if (cm > 0 && cm <= stopCm / 2) {
                return true;
            }
        }
        return false;
    }

    public int nearestFrontCm() {
        int best = Integer.MAX_VALUE;
        for (int part : FRONT_PARTS) {
            int idx = irDistances.indexOfKey(part);
            if (idx >= 0) {
                int cm = irDistances.valueAt(idx);
                if (cm > 0 && cm < best) {
                    best = cm;
                }
            }
        }
        return best == Integer.MAX_VALUE ? -1 : best;
    }

    private void updateStatusLine() {
        if (obstacleActive) {
            statusLine = "OBSTACLE STOP";
            statusPaint.setColor(0xFFF87171);
            return;
        }
        int front = nearestFrontCm();
        if (front > 0 && front <= 40) {
            statusLine = "FRONT " + front + " cm (stand-off)";
            statusPaint.setColor(0xFFFBBF24);
            return;
        }
        if (isSensorsLive()) {
            int n = nearestFrontCm();
            statusLine = n > 0 ? ("IR live · front " + n + " cm") : "IR live · clear";
            statusPaint.setColor(0xFF5EEAD4);
        } else {
            statusLine = "waiting for IR…";
            statusPaint.setColor(0xFF94A3B8);
        }
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        float w = getWidth();
        float h = getHeight();
        if (w <= 0 || h <= 0) {
            return;
        }

        float cx = w / 2f;
        float cy = h * 0.52f;
        float radius = Math.min(w, h) * 0.38f;

        // Background disc
        bodyPaint.setColor(0xFF0B1220);
        canvas.drawCircle(cx, cy, radius * 1.05f, bodyPaint);

        for (float ring : new float[]{0.33f, 0.66f, 1.0f}) {
            canvas.drawCircle(cx, cy, radius * ring, ringPaint);
        }

        // Cross hairs
        canvas.drawLine(cx - radius, cy, cx + radius, cy, ringPaint);
        canvas.drawLine(cx, cy - radius, cx, cy + radius, ringPaint);

        if (pirFront) {
            wedgePaint.setColor(0x38FB923C);
            drawWedge(canvas, cx, cy, radius * 0.92f, -50, 50);
        }
        if (pirBack) {
            wedgePaint.setColor(0x38FB923C);
            drawWedge(canvas, cx, cy, radius * 0.92f, 130, 230);
        }

        int count = irDistances.size();
        for (int i = 0; i < count; i++) {
            int part = irDistances.keyAt(i);
            int cm = irDistances.valueAt(i);
            if (cm <= 0) {
                continue;
            }
            double bearing = bearingForPart(part, i, count);
            // Screen: 0° front = up → convert bearing (CW from front) to canvas radians
            double rad = Math.toRadians(bearing) - Math.PI / 2.0;
            float clamped = Math.min(MAX_RANGE_CM, Math.max(0, cm));
            float dist = radius * (clamped / MAX_RANGE_CM);
            float ex = cx + (float) (Math.cos(rad) * dist);
            float ey = cy + (float) (Math.sin(rad) * dist);

            boolean hot = cm <= 25;
            boolean warm = cm <= 40;
            rayPaint.setStrokeWidth(hot ? 4f : 2.5f);
            rayPaint.setColor(hot ? 0xDDF87171 : (warm ? 0xDDFBBF24 : 0xBB5EEAD4));
            canvas.drawLine(cx, cy, ex, ey, rayPaint);

            blipPaint.setColor(hot ? 0xFFF87171 : (warm ? 0xFFFBBF24 : 0xFF5EEAD4));
            canvas.drawCircle(ex, ey, hot ? 7f : 5f, blipPaint);
        }

        // Robot body — red when obstacle stop active
        bodyPaint.setColor(obstacleActive ? 0xDDF87171 : 0xEE1E293B);
        RectF body = new RectF(cx - 18, cy - 24, cx + 18, cy + 24);
        canvas.drawRoundRect(body, 10, 10, bodyPaint);

        tmpPath.reset();
        tmpPath.moveTo(cx, cy - 34);
        tmpPath.lineTo(cx - 8, cy - 20);
        tmpPath.lineTo(cx + 8, cy - 20);
        tmpPath.close();
        canvas.drawPath(tmpPath, nosePaint);

        canvas.drawText("RADAR", 12, 28, labelPaint);
        canvas.drawText(statusLine, 12, 56, statusPaint);

        String live = isSensorsLive() ? "● LIVE" : "○ NO DATA";
        labelPaint.setColor(isSensorsLive() ? 0xFF5EEAD4 : 0xFF64748B);
        canvas.drawText(live, w - labelPaint.measureText(live) - 12, 28, labelPaint);
        labelPaint.setColor(0xFF94A3B8);
    }

    private void drawWedge(Canvas canvas, float cx, float cy, float r, float startDeg, float endDeg) {
        // Convert bearing degrees (0=front/up, CW) to Android arc (0=east, CW) relative to oval
        // Android drawArc: 0° = right, positive = clockwise. Front-up = -90°.
        float sweep = endDeg - startDeg;
        float androidStart = startDeg - 90f;
        RectF oval = new RectF(cx - r, cy - r, cx + r, cy + r);
        canvas.drawArc(oval, androidStart, sweep, true, wedgePaint);
    }

    /**
     * Approximate bearing for IR part ids. Unknown ids fan out evenly.
     * 0° = front, clockwise.
     */
    private static double bearingForPart(int part, int index, int count) {
        switch (part) {
            case 1: return 0;
            case 2: return 20;
            case 3: return 340;
            case 4: return 40;
            case 5: return 320;
            case 6: return 60;
            case 7: return 300;
            case 8: return 90;
            case 9: return 270;
            case 10: return 120;
            case 11: return 240;
            case 12: return 150;
            case 13: return 210;
            case 14: return 180;
            default:
                if (count <= 0) {
                    return 0;
                }
                return (360.0 * index) / count;
        }
    }
}
