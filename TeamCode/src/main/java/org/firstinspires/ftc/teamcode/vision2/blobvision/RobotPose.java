package org.firstinspires.ftc.teamcode.vision2.blobvision;

/**
 * Minimal robot pose (field frame, inches + radians, CCW+ heading, Pedro convention).
 * Keeps the vision classes independent of any path-following library.
 */
public final class RobotPose {
    public final double x;
    public final double y;
    public final double heading;

    public RobotPose(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.heading = headingRad;
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.1f deg)", x, y, Math.toDegrees(heading));
    }
}
