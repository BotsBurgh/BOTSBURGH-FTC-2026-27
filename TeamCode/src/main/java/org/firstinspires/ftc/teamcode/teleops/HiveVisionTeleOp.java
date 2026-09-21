package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Vision Test")
public class HiveVisionTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        HiveVisionLimelight vision =
                new HiveVisionLimelight(hardwareMap);

        telemetry.addLine("Hive Vision Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            HiveVisionLimelight.Detection yellow =
                    vision.getYellowPollen();

            HiveVisionLimelight.Detection red =
                    vision.getRedNectar();

            HiveVisionLimelight.Detection blue =
                    vision.getBlueNectar();

            telemetry.addLine("===== HIVE VISION =====");

            telemetry.addData(
                    "Yellow",
                    yellow.visible
                            ? "%.1f%%  tx=%.2f ty=%.2f"
                            : "NOT SEEN",
                    yellow.confidence * 100,
                    yellow.tx,
                    yellow.ty
            );

            telemetry.addData(
                    "Red",
                    red.visible
                            ? "%.1f%%  tx=%.2f ty=%.2f"
                            : "NOT SEEN",
                    red.confidence * 100,
                    red.tx,
                    red.ty
            );

            telemetry.addData(
                    "Blue",
                    blue.visible
                            ? "%.1f%%  tx=%.2f ty=%.2f"
                            : "NOT SEEN",
                    blue.confidence * 100,
                    blue.tx,
                    blue.ty
            );

            telemetry.update();
        }

        vision.stop();
    }
}