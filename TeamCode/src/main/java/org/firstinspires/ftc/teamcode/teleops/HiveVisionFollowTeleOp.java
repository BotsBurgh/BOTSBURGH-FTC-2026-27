package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveVisionLimelight;

@TeleOp(name = "Hive Vision Follow")
public class HiveVisionFollowTeleOp extends OpMode {

    private Robot robot;

    private static final double TURN_GAIN = 0.025;
    private static final double MAX_TURN = 0.45;
    private static final double DEADZONE = 2.0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        robot.update();

        HiveVisionLimelight.Detection yellow = robot.vision.getYellowPollen();

        telemetry.addData("LL", robot.vision.debugClasses());

        if (!yellow.visible) {

            robot.drive.stop();
            telemetry.addLine("NO YELLOW");

        } else {

            double tx = yellow.tx;

            if (Math.abs(tx) < DEADZONE) {
                tx = 0;
            }

            double turn = tx * TURN_GAIN;
            turn = Math.max(-MAX_TURN, Math.min(MAX_TURN, turn));

            double forward = 0.20;
            if (Math.abs(yellow.tx) > 20) {
                forward = 0.10;
            }

            robot.drive.drive(forward, turn);

            telemetry.addData("TX", yellow.tx);
            telemetry.addData("Confidence", yellow.confidence * 100);
            telemetry.addData("Turn", turn);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}