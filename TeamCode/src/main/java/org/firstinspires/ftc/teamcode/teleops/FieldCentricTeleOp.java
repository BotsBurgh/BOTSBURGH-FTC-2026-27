package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

@TeleOp(name = "Field Centric Drive", group = "Drive")
public class FieldCentricTeleOp extends OpMode {

    private Drivetrain drive;
    private IMU imu;

    private static final double SLOW_MODE_SCALE = 0.4;

    // SET THESE to match how the Control Hub is mounted on the robot
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)));
        imu.resetYaw();
    }

    @Override
    public void loop() {

        // Options button: point the robot away from you, press to zero heading
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double y = -gamepad1.left_stick_y;   // forward on the field
        double x = gamepad1.left_stick_x;    // right on the field
        double turn = gamepad1.right_stick_x;

        // Rotate the stick vector by -heading to get robot-relative motion
        double forward = x * Math.sin(-heading) + y * Math.cos(-heading);
        double strafe = x * Math.cos(-heading) - y * Math.sin(-heading);

        double scale = gamepad1.left_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;

        drive.drive(forward * scale, strafe * scale, turn * scale);

        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("Slow mode", scale < 1.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}