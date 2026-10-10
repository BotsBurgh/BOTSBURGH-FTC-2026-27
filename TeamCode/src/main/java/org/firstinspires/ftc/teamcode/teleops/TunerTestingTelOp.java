
package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "TunerTesting")
public class TunerTestingTelOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            robot.shooter.moveToTick(500);
        } else if (gamepad1.dpad_down) {
            robot.shooter.moveToTick(0);
        }

        if (gamepad1.a) {
            robot.shooter.runCounterRoller();
        } else {
            robot.shooter.stopCounterRoller();
        }

        robot.update();

        telemetry.addData("Shooter Ticks", robot.shooter.getTurretTicks());
        telemetry.addData("Turret Target", robot.shooter.getTurretTarget());
        telemetry.addData("Turret Velocity", robot.shooter.getTurretVelocity());
        telemetry.addData("Counter Roller Velocity", robot.shooter.getCounterRollerVelocity());
        telemetry.addData("Aimer 1 Position", robot.shooter.getAimer1Position());
        telemetry.addData("Aimer 2 Position", robot.shooter.getAimer2Position());

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
