
package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "TunerTesting")
public class TunerTestingTelOp extends OpMode {

    private Robot robot;
    private TelemetryManager panels;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        ShooterConfig.ENABLED = false;
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            ShooterConfig.ENABLED = false;
        }

        robot.update();

        double target = robot.shooter.getTurretTargetVelocity();
        double actual = robot.shooter.getTurretVelocity();

        double rollerTarget = ShooterConfig.ENABLED
                ? ShooterConfig.TARGET_VELOCITY
                * ShooterConfig.ROLLER_TURRET_RATIO
                : 0.0;

        double rollerActual = robot.shooter.getCounterRollerVelocity();

        panels.debug("Enabled", ShooterConfig.ENABLED);
        panels.debug("Target Velocity", target);
        panels.debug("Actual Velocity", actual);
        panels.debug("Velocity Error", target - actual);
        panels.debug("Turret Ticks", robot.shooter.getTurretTicks());

        panels.debug("Counter Roller Target", rollerTarget);
        panels.debug("Counter Roller Actual", rollerActual);

        panels.debug("P", ShooterConfig.TURRET_P);
        panels.debug("I", ShooterConfig.TURRET_I);
        panels.debug("D", ShooterConfig.TURRET_D);
        panels.debug("F", ShooterConfig.TURRET_F);
        panels.debug("Roller Ratio", ShooterConfig.ROLLER_TURRET_RATIO);

        panels.update(telemetry);
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
