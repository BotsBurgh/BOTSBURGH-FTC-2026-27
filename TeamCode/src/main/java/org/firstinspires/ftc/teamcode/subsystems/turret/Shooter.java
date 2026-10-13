
package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;

public class Shooter {

    private final DcMotorEx turret;
    private final DcMotorEx counterRoller;

    public Shooter(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        counterRoller = hardwareMap.get(DcMotorEx.class, "counterRoller");

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counterRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runTurret() {
        updateTurretPIDF();

        if (ShooterConfig.ENABLED) {
            double target = ShooterConfig.TARGET_VELOCITY;

            turret.setVelocity(target);
            counterRoller.setVelocity(
                    target * ShooterConfig.ROLLER_TURRET_RATIO
            );
        } else {
            turret.setPower(0);
            counterRoller.setPower(0);
        }
    }

    private void updateTurretPIDF() {
        turret.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        ShooterConfig.TURRET_P,
                        ShooterConfig.TURRET_I,
                        ShooterConfig.TURRET_D,
                        ShooterConfig.TURRET_F
                )
        );
    }

    public double getTurretTargetVelocity() {
        return ShooterConfig.ENABLED
                ? ShooterConfig.TARGET_VELOCITY : 0.0;
    }

    public double getTurretVelocity() {
        return turret.getVelocity();
    }

    public double getCounterRollerVelocity() {
        return counterRoller.getVelocity();
    }

    public int getTurretTicks() {
        return turret.getCurrentPosition();
    }

    public void stop() {
        turret.setPower(0);
        counterRoller.setPower(0);
    }
}
