package org.firstinspires.ftc.teamcode.vision2.blobvision;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

/**
 * BlobDrive backed by Pedro Pathing 3's {@code follower.manual(...)}.
 *
 * Your OpMode must still call {@code follower.update()} every loop (this class does not).
 *
 * Pedro 3's docs feed raw gamepad values to manual() (right = +lateral, right-stick-right = +turn),
 * so by default we flip our left/CCW commands. If the robot strafes or turns the wrong way in
 * BlobCalibrationTeleOp, flip lateralSign / turnSign (that is the only tweak needed).
 */
public final class PedroBlobDrive implements BlobDrive {

    private final Follower follower;

    public double forwardSign = 1.0;
    public double lateralSign = -1.0;
    public double turnSign = -1.0;

    public PedroBlobDrive(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void drive(double forward, double left, double turnCcw) {
        follower.manual(forward * forwardSign, left * lateralSign, turnCcw * turnSign);
    }

    @Override
    public void stop() {
        follower.manual(0, 0, 0);
    }

    @Override
    public RobotPose pose() {
        Pose p = follower.pose();
        return new RobotPose(p.x(), p.y(), p.heading());
    }
}
