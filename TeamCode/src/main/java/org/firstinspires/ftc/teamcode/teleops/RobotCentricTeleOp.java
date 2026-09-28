package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

@TeleOp(name = "Robot Centric Drive", group = "Drive")
public class RobotCentricTeleOp extends OpMode {

    private Drivetrain drive;

    private static final double SLOW_MODE_SCALE = 0.4;

    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double scale = gamepad1.a? SLOW_MODE_SCALE : 1.0;

        drive.drive(forward * scale, strafe * scale, turn * scale);

        telemetry.addData("Slow mode", scale < 1.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}