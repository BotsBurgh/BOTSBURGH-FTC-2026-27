package org.firstinspires.ftc.teamcode.vision2.blobvision;

/**
 * Whatever moves your robot. Implement this for your own drivetrain, or use {@link PedroBlobDrive}.
 * Convention: forward +, left +, turn CCW+, all -1..1, robot-centric.
 */
public interface BlobDrive {

    void drive(double forward, double left, double turnCcw);

    /** Command zero motion. */
    void stop();

    /** Current field pose, or null if you have no localization (field coordinates are then skipped). */
    RobotPose pose();

    default void drive(BlobControl.Command c) {
        drive(c.forward, c.left, c.turn);
    }
}
