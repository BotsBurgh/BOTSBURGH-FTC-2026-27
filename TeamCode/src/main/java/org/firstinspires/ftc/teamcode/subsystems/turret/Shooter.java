        package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;

public class Shooter {

    private final DcMotorEx turret;
    private final DcMotorEx counterRoller;

    public double turretPower = -0.48;
    public double crpower = -1.0;

    public double turretTargetVelocity = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        counterRoller = hardwareMap.get(DcMotorEx.class, "counterRoller");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        counterRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updateTurretPIDF();
    }

    public void updateTurretPIDF() {
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

    public void setTurretVelocity(double velocity) {
        turretTargetVelocity = velocity;
        turret.setVelocity(velocity);
    }

    public void runTurret() {
        updateTurretPIDF();
        turret.setVelocity(turretTargetVelocity);
    }

    public void runCR() {
        counterRoller.setPower(crpower);
    }

    public void runCounterRoller() {
        double rollerVelocity =
                turret.getVelocity() * ShooterConfig.ROLLER_TURRET_RATIO;

        counterRoller.setVelocity(rollerVelocity);
    }

    public void stopCounterRoller() {
        counterRoller.setPower(0);
    }

    public int getTurretTicks() {
        return turret.getCurrentPosition();
    }

    public double getTurretVelocity() {
        return turret.getVelocity();
    }

    public double getCounterRollerVelocity() {
        return counterRoller.getVelocity();
    }

    public int getTurretTarget() {
        return turret.getTargetPosition();
    }

    public double getTurretTargetVelocity() {
        return turretTargetVelocity;
    }

    public void addTelemetry(
            com.qualcomm.robotcore.eventloop.opmode.OpMode opMode) {

        opMode.telemetry.addData("Turret Ticks", getTurretTicks());
        opMode.telemetry.addData("Turret Velocity", getTurretVelocity());
        opMode.telemetry.addData(
                "Turret Target Velocity", getTurretTargetVelocity());

        opMode.telemetry.addData("Turret P", ShooterConfig.TURRET_P);
        opMode.telemetry.addData("Turret I", ShooterConfig.TURRET_I);
        opMode.telemetry.addData("Turret D", ShooterConfig.TURRET_D);
        opMode.telemetry.addData("Turret F", ShooterConfig.TURRET_F);

        opMode.telemetry.addData(
                "Counter Roller Velocity", getCounterRollerVelocity());
    }

    public void update() {
        runTurret();
        runCR();
    }

    public void stop() {
        turret.setVelocity(0);
        counterRoller.setPower(0);
    }
}
