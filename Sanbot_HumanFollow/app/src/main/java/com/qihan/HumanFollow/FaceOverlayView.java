package com.qihan.HumanFollow;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.View;

import java.util.ArrayList;
import java.util.List;

/**
 * Draws person/face boxes. Locked target uses thick teal; others are thinner white/amber.
 */
public class FaceOverlayView extends View {

    private final Paint otherPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint personPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint targetPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelBgPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private final List<RectF> otherBoxes = new ArrayList<>();
    private final List<String> otherLabels = new ArrayList<>();
    private RectF targetBox;
    private String targetLabel = "LOCK";

    private float frameWidth = 1280f;
    private float frameHeight = 720f;

    public FaceOverlayView(Context context) {
        super(context);
        init();
    }

    public FaceOverlayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public FaceOverlayView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        setWillNotDraw(false);
        otherPaint.setStyle(Paint.Style.STROKE);
        otherPaint.setStrokeWidth(3f);
        otherPaint.setColor(Color.WHITE);

        personPaint.setStyle(Paint.Style.STROKE);
        personPaint.setStrokeWidth(3.5f);
        personPaint.setColor(0xFFFBBF24);

        targetPaint.setStyle(Paint.Style.STROKE);
        targetPaint.setStrokeWidth(6f);
        targetPaint.setColor(0xFF5EEAD4);

        labelPaint.setColor(Color.BLACK);
        labelPaint.setTextSize(26f);
        labelPaint.setFakeBoldText(true);

        labelBgPaint.setStyle(Paint.Style.FILL);
        labelBgPaint.setColor(0xFF5EEAD4);
    }

    public void setFrameSize(int width, int height) {
        if (width > 0 && height > 0) {
            frameWidth = width;
            frameHeight = height;
        }
    }

    public void clearFaces() {
        otherBoxes.clear();
        otherLabels.clear();
        targetBox = null;
        postInvalidate();
    }

    public void setTargets(List<TrackTarget> targets, TrackTarget locked) {
        otherBoxes.clear();
        otherLabels.clear();
        targetBox = null;
        if (targets == null || targets.isEmpty()) {
            if (locked != null) {
                targetBox = toViewRect(locked);
                targetLabel = locked.label != null ? locked.label : "LOCK";
            }
            postInvalidate();
            return;
        }
        for (TrackTarget t : targets) {
            RectF rect = toViewRect(t);
            if (rect == null) {
                continue;
            }
            if (locked != null && sameBox(t, locked)) {
                targetBox = rect;
                targetLabel = locked.kind == TrackTarget.Kind.FACE ? "FACE LOCK"
                        : (locked.label != null ? locked.label + " LOCK" : "PERSON LOCK");
            } else {
                otherBoxes.add(rect);
                otherLabels.add(t.kind == TrackTarget.Kind.FACE ? "face" : t.label);
            }
        }
        if (locked != null && targetBox == null) {
            targetBox = toViewRect(locked);
            targetLabel = "LOCK";
        }
        postInvalidate();
    }

    private static boolean sameBox(TrackTarget a, TrackTarget b) {
        return a.kind == b.kind
                && Math.abs(a.centerX() - b.centerX()) < 0.02
                && Math.abs(a.centerY() - b.centerY()) < 0.02;
    }

    private RectF toViewRect(TrackTarget t) {
        if (t == null) {
            return null;
        }
        float viewW = getWidth();
        float viewH = getHeight();
        if (viewW <= 0 || viewH <= 0) {
            return null;
        }
        return new RectF(
                (float) (t.left * viewW),
                (float) (t.top * viewH),
                (float) (t.right * viewW),
                (float) (t.bottom * viewH));
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        for (int i = 0; i < otherBoxes.size(); i++) {
            RectF rect = otherBoxes.get(i);
            String lab = otherLabels.get(i);
            boolean person = lab != null && !lab.equalsIgnoreCase("face");
            canvas.drawRect(rect, person ? personPaint : otherPaint);
        }
        if (targetBox != null) {
            canvas.drawRect(targetBox, targetPaint);
            float pad = 8f;
            float textW = labelPaint.measureText(targetLabel);
            float labelTop = Math.max(0, targetBox.top - 36);
            canvas.drawRect(
                    targetBox.left,
                    labelTop,
                    targetBox.left + textW + pad * 2,
                    labelTop + 34,
                    labelBgPaint);
            canvas.drawText(targetLabel, targetBox.left + pad, labelTop + 26, labelPaint);
        }
    }
}
