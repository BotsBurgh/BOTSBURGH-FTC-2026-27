package org.firstinspires.ftc.teamcode.vision2.blobvision;

/**
 * One detected artifact, already converted to useful robot-frame numbers.
 * Immutable. Robot frame: +forward, +left (inches); bearing is degrees, + = left of forward.
 * Range fields are NaN when no distance could be computed (see {@link #hasRange()}).
 */
public final class Blob {

    public final BlobColor color;

    // raw Limelight values
    public final double txDeg;    // + = right of crosshair
    public final double tyDeg;    // + = above crosshair
    public final double areaPct;
    public final double confidence;

    // derived
    public final double cameraDistanceIn; // floor distance from the camera lens
    public final double distanceIn;       // floor distance from robot center
    public final double forwardIn;
    public final double leftIn;
    public final double bearingDeg;       // robot-frame, + = left

    // field frame (NaN until a pose is supplied)
    public final double fieldX;
    public final double fieldY;

    public final long timestampMs;

    public Blob(BlobColor color, double txDeg, double tyDeg, double areaPct, double confidence,
                double cameraDistanceIn, double distanceIn, double forwardIn, double leftIn,
                double bearingDeg, double fieldX, double fieldY, long timestampMs) {
        this.color = color;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.areaPct = areaPct;
        this.confidence = confidence;
        this.cameraDistanceIn = cameraDistanceIn;
        this.distanceIn = distanceIn;
        this.forwardIn = forwardIn;
        this.leftIn = leftIn;
        this.bearingDeg = bearingDeg;
        this.fieldX = fieldX;
        this.fieldY = fieldY;
        this.timestampMs = timestampMs;
    }

    public boolean hasRange() {
        return !Double.isNaN(distanceIn);
    }

    public boolean hasField() {
        return !Double.isNaN(fieldX) && !Double.isNaN(fieldY);
    }

    public Blob withField(double fx, double fy) {
        return new Blob(color, txDeg, tyDeg, areaPct, confidence, cameraDistanceIn, distanceIn,
                forwardIn, leftIn, bearingDeg, fx, fy, timestampMs);
    }

    /** Exponential blend toward a newer sighting of (what we believe is) the same blob. */
    public Blob blend(Blob n, double alpha) {
        return new Blob(n.color,
                mix(txDeg, n.txDeg, alpha), mix(tyDeg, n.tyDeg, alpha),
                mix(areaPct, n.areaPct, alpha), mix(confidence, n.confidence, alpha),
                mix(cameraDistanceIn, n.cameraDistanceIn, alpha), mix(distanceIn, n.distanceIn, alpha),
                mix(forwardIn, n.forwardIn, alpha), mix(leftIn, n.leftIn, alpha),
                mix(bearingDeg, n.bearingDeg, alpha),
                mix(fieldX, n.fieldX, alpha), mix(fieldY, n.fieldY, alpha),
                n.timestampMs);
    }

    /** Distance between two sightings: field frame if both have it, else robot frame, else bearing. */
    public double separationTo(Blob o) {
        if (hasField() && o.hasField()) return Math.hypot(fieldX - o.fieldX, fieldY - o.fieldY);
        if (hasRange() && o.hasRange()) return Math.hypot(forwardIn - o.forwardIn, leftIn - o.leftIn);
        return Math.abs(bearingDeg - o.bearingDeg);
    }

    /** X used for planning: field X when planning in the field frame, otherwise robot forward. */
    public double planX(boolean field) {
        return field ? fieldX : forwardIn;
    }

    /** Y used for planning: field Y when planning in the field frame, otherwise robot left. */
    public double planY(boolean field) {
        return field ? fieldY : leftIn;
    }

    private static double mix(double old, double nw, double a) {
        if (Double.isNaN(nw)) return old;
        if (Double.isNaN(old)) return nw;
        return old + a * (nw - old);
    }

    @Override
    public String toString() {
        return String.format("%s dist=%.1fin bearing=%.1fdeg tx=%.1f ty=%.1f area=%.2f%%",
                color, distanceIn, bearingDeg, txDeg, tyDeg, areaPct);
    }
}
