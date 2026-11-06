package com.qihan.HumanFollow;

/** Normalized (0..1) person/face track box for overlay + follow. */
public final class TrackTarget {
    public enum Kind {
        FACE,
        PERSON
    }

    public final Kind kind;
    public final double left;
    public final double top;
    public final double right;
    public final double bottom;
    public final String label;
    /**
     * Normalized height used for stand-off / approach (0..1).
     * Face-expanded body boxes keep the original face height here so follow
     * distance is not inflated by the torso expansion.
     */
    public final double rangeHeight;

    public TrackTarget(Kind kind, double left, double top, double right, double bottom, String label) {
        this(kind, left, top, right, bottom, label, -1);
    }

    public TrackTarget(Kind kind, double left, double top, double right, double bottom,
                       String label, double rangeHeight) {
        this.kind = kind;
        this.left = clamp01(Math.min(left, right));
        this.top = clamp01(Math.min(top, bottom));
        this.right = clamp01(Math.max(left, right));
        this.bottom = clamp01(Math.max(top, bottom));
        this.label = label != null ? label : (kind == Kind.FACE ? "FACE" : "PERSON");
        double boxH = Math.max(0.01, this.bottom - this.top);
        this.rangeHeight = rangeHeight > 0 ? clamp01(rangeHeight) : boxH;
    }

    public double centerX() {
        return (left + right) / 2.0;
    }

    public double centerY() {
        return (top + bottom) / 2.0;
    }

    public double width() {
        return Math.max(0.01, right - left);
    }

    public double height() {
        return Math.max(0.01, bottom - top);
    }

    public double area() {
        return width() * height();
    }

    /** Expand a face box into an approximate upper-body / torso box. */
    public static TrackTarget bodyFromFace(TrackTarget face) {
        double fh = face.height();
        double fw = face.width();
        double cx = face.centerX();
        double bodyH = Math.min(0.95, fh * 4.2);
        double bodyW = Math.min(0.75, Math.max(fw * 2.2, fh * 1.6));
        double top = Math.max(0, face.top - fh * 0.15);
        double bottom = Math.min(1.0, top + bodyH);
        double left = Math.max(0, cx - bodyW / 2.0);
        double right = Math.min(1.0, cx + bodyW / 2.0);
        // Keep face height as range proxy — bodyH alone looks "close enough" and stalls follow.
        return new TrackTarget(Kind.PERSON, left, top, right, bottom, "PERSON", fh);
    }

    private static double clamp01(double v) {
        if (v < 0) {
            return 0;
        }
        if (v > 1) {
            return 1;
        }
        return v;
    }
}
