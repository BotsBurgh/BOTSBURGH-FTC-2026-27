package org.firstinspires.ftc.teamcode.vision2.blobvision;

/**
 * Pure geometry - no hardware. Safe to call from anywhere (TeleOp, Auto, unit tests).
 */
public final class BlobMath {

    private BlobMath() {}

    public static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** Wrap to (-pi, pi]. */
    public static double angleWrap(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad <= -Math.PI) rad += 2 * Math.PI;
        return rad;
    }

    /**
     * Projects a Limelight (tx, ty) ray onto the floor plane.
     * Pinhole model: ray = (tan tx, tan ty, 1) in camera axes, camera pitched down by pitchDown.
     *
     * @return {forward, left} in inches relative to the lens, along camera-yaw-corrected robot axes
     *         (not yet offset by the lens position), or null if the ray never hits the floor plane
     */
    public static double[] groundFromAngles(BlobConfig c, double txDeg, double tyDeg) {
        double tx = Math.tan(Math.toRadians(txDeg));
        double ty = Math.tan(Math.toRadians(tyDeg));
        double th = Math.toRadians(c.cameraPitchDownDeg);

        double rayFwd = Math.cos(th) + ty * Math.sin(th);
        double rayUp = -Math.sin(th) + ty * Math.cos(th);
        double rayRight = tx;

        double dz = c.blobCenterHeightIn - c.cameraHeightIn; // negative when camera is above the blob
        if (rayUp >= -1e-6 || dz >= 0) return null;          // looking at or above the horizon

        double s = dz / rayUp;
        double fwd = s * rayFwd;
        double left = -s * rayRight;
        return yaw(c, fwd, left);
    }

    /** Area fallback: distance = k / sqrt(area%), direction from tx. Same return format as above. */
    public static double[] groundFromArea(BlobConfig c, double txDeg, double areaPct) {
        if (c.areaConstantK <= 0 || areaPct <= 0) return null;
        double d = c.areaConstantK / Math.sqrt(areaPct);
        double tx = Math.toRadians(txDeg);
        return yaw(c, d * Math.cos(tx), -d * Math.sin(tx));
    }

    private static double[] yaw(BlobConfig c, double fwd, double left) {
        double y = Math.toRadians(c.cameraYawDeg);
        return new double[]{
                fwd * Math.cos(y) - left * Math.sin(y),
                fwd * Math.sin(y) + left * Math.cos(y)
        };
    }

    /** Turns raw Limelight numbers into a fully populated {@link Blob}. */
    public static Blob makeBlob(BlobConfig c, BlobColor color, double txDeg, double tyDeg,
                                double areaPct, double confidence, long nowMs) {
        double[] g = null;
        if (c.distanceMode == BlobConfig.DistanceMode.ANGLE) {
            g = groundFromAngles(c, txDeg, tyDeg);
            if (g == null) g = groundFromArea(c, txDeg, areaPct); // fallback
        } else {
            g = groundFromArea(c, txDeg, areaPct);
            if (g == null) g = groundFromAngles(c, txDeg, tyDeg);
        }

        if (g == null) {
            // No range available: still usable for aiming.
            double bearing = -txDeg + c.cameraYawDeg;
            return new Blob(color, txDeg, tyDeg, areaPct, confidence,
                    Double.NaN, Double.NaN, Double.NaN, Double.NaN, bearing,
                    Double.NaN, Double.NaN, nowMs);
        }

        double camDist = Math.hypot(g[0], g[1]);
        double fwd = g[0] + c.cameraForwardOffsetIn;
        double left = g[1] + c.cameraLeftOffsetIn;
        double dist = Math.hypot(fwd, left);
        double bearing = Math.toDegrees(Math.atan2(left, fwd));
        return new Blob(color, txDeg, tyDeg, areaPct, confidence,
                camDist, dist, fwd, left, bearing, Double.NaN, Double.NaN, nowMs);
    }

    /** Robot-frame point -> field frame. Returns {x, y}. */
    public static double[] toField(RobotPose p, double forwardIn, double leftIn) {
        double cos = Math.cos(p.heading);
        double sin = Math.sin(p.heading);
        return new double[]{
                p.x + forwardIn * cos - leftIn * sin,
                p.y + forwardIn * sin + leftIn * cos
        };
    }

    /** Camera pitch (deg) that makes a blob at ty appear exactly knownGroundDistanceIn from the lens. */
    public static double suggestPitchDownDeg(BlobConfig c, double tyDeg, double knownGroundDistanceIn) {
        double below = Math.toDegrees(Math.atan((c.cameraHeightIn - c.blobCenterHeightIn) / knownGroundDistanceIn));
        return below + tyDeg;
    }

    /** Area constant K so that K / sqrt(area) == knownGroundDistanceIn. */
    public static double suggestAreaK(double areaPct, double knownGroundDistanceIn) {
        return knownGroundDistanceIn * Math.sqrt(areaPct);
    }
}
