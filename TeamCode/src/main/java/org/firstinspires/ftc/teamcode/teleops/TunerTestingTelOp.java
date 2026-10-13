        package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;

@TeleOp(name = "TunerTesting")
public class TunerTestingTelOp extends OpMode {

    private Robot robot;
    private double targetVelocity = 0.0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            targetVelocity += 10;
        }

        if (gamepad1.dpad_down) {
            targetVelocity -= 10;
        }

        if (gamepad1.a) {
            targetVelocity = 0;
        }

        robot.shooter.setTurretVelocity(targetVelocity);
        robot.update();

        telemetry.addData(
                "Turret Ticks", robot.shooter.getTurretTicks());
        telemetry.addData(
                "Target Velocity", targetVelocity);
        telemetry.addData(
                "Actual Velocity", robot.shooter.getTurretVelocity());

        telemetry.addData("P", ShooterConfig.TURRET_P);
        telemetry.addData("I", ShooterConfig.TURRET_I);
        telemetry.addData("D", ShooterConfig.TURRET_D);
        telemetry.addData("F", ShooterConfig.TURRET_F);

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
