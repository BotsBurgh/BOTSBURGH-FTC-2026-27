package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Adjust to wherever your Pedro Constants class lives
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * FieldCentricTeleOp - Field-centric mecanum drive using Pedro Pathing 3.
 *
 * Controls (gamepad1):
 *   - Left Stick   : Field-relative translation (up = away from driver)
 *   - Right Stick X: Rotation
 *   - Left Trigger : Slow mode (0.4x)
 *   - Options / Back / Start : Re-zero field heading (point robot away from you first)
 *
 * Heading comes from Pedro's localizer, so no separate IMU is needed.
 */
@TeleOp(name = "pedrofieldCentric", group = "Drive")
public class pedroTeleop extends OpMode {

    private static final double SLOW_MODE_SCALE = 0.4;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double DEADBAND = 0.05;

    // Hold position automatically when the sticks are released
    private static final boolean HOLD_ON_RELEASE = false;

    // Used if you don't carry a pose over from auto
    private static final Pose START_POSE = new Pose(0, 0, 0);

    private Follower follower;

    // Heading (rad) that counts as "field forward" for the driver
    private double headingOffset = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
        follower.update();
    }

    @Override
    public void start() {
        // If you pass a pose from auto, set it here: follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
        headingOffset = follower.pose().heading();
    }

    @Override
    public void loop() {
        // Re-zero field heading without touching the localizer pose
        if (gamepad1.options || gamepad1.back || gamepad1.start) {
            headingOffset = follower.pose().heading();
        }

        double scale = gamepad1.left_trigger > TRIGGER_THRESHOLD ? SLOW_MODE_SCALE : 1.0;

        double forward = deadband(-gamepad1.left_stick_y) * scale;
        double lateral = deadband(gamepad1.left_stick_x) * scale;
        double turn = deadband(gamepad1.right_stick_x) * scale;

        double fieldHeading = follower.pose().heading() - headingOffset;

        DrivePowers powers = ManualDrive.fieldCentric(forward, lateral, turn, fieldHeading);

        if (HOLD_ON_RELEASE) {
            ManualDrive.driveOrHold(follower, powers);
        } else {
            follower.manual(powers);
        }

        follower.update();

        Pose pose = follower.pose();
        telemetry.addData("X (in)", "%.1f", pose.x());
        telemetry.addData("Y (in)", "%.1f", pose.y());
        telemetry.addData("Field Heading (deg)", "%.1f", Math.toDegrees(fieldHeading));
        telemetry.addData("Slow Mode", scale < 1.0);
        telemetry.update();
    }

    private static double deadband(double v) {
        return Math.abs(v) < DEADBAND ? 0 : v;
    }
}