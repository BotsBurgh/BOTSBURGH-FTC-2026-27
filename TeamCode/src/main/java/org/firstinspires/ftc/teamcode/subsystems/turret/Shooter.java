package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;

public class Shooter {
    private final DcMotorEx turret;
    private final DcMotorEx counterRoller;
    private final Servo aimer1;
    private final Servo aimer2;

    private DigitalChannel encoderA;
    private DigitalChannel encoderB;

    public Shooter(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        counterRoller = hardwareMap.get(DcMotorEx.class, "counterRoller");

        aimer1 = hardwareMap.get(Servo.class, "aimer1");
        aimer2 = hardwareMap.get(Servo.class, "aimer2");

        encoderA = hardwareMap.get(DigitalChannel.class, "encoderA");
        encoderB = hardwareMap.get(DigitalChannel.class, "encoderB");

        encoderA.setMode(DigitalChannel.Mode.INPUT);
        encoderB.setMode(DigitalChannel.Mode.INPUT);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0);

        counterRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updatePIDF();
    }

    public void updatePIDF() {
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(ShooterConfig.TURRET_P, ShooterConfig.TURRET_I, ShooterConfig.TURRET_D, ShooterConfig.TURRET_F));

        counterRoller.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(ShooterConfig.ROLLER_P, ShooterConfig.ROLLER_I, ShooterConfig.ROLLER_D, ShooterConfig.ROLLER_F));
    }

    public void moveToTick(int ticks) {
        turret.setTargetPosition(ticks);
        turret.setPower(ShooterConfig.TURRET_POWER);
    }

    public void runCounterRoller() {
        double turretVelocity = turret.getVelocity();
        double rollerVelocity = turretVelocity * ShooterConfig.ROLLER_TURRET_RATIO;

        counterRoller.setVelocity(rollerVelocity);
    }

    public void stopCounterRoller() {
        counterRoller.setVelocity(0);
    }

    public int getTurretTicks() {
        return turret.getCurrentPosition();
    }

    public void update() {
        updatePIDF();
        runCounterRoller();
    }
    public double getTurretVelocity() {
        return turret.getVelocity();
    }

    public double getCounterRollerVelocity() {
        return counterRoller.getVelocity();
    }

    public double getAimer1Position() {
        return aimer1.getPosition();
    }

    public double getAimer2Position() {
        return aimer2.getPosition();
    }
    public int getTurretTarget() {
        return turret.getTargetPosition();
    }

    public void addTelemetry(com.qualcomm.robotcore.eventloop.opmode.OpMode opMode) {
        opMode.telemetry.addData("Shooter Ticks", getTurretTicks());
        opMode.telemetry.addData("Shooter Target", turret.getTargetPosition());
        opMode.telemetry.addData("Counter Roller Velocity", counterRoller.getVelocity());
        opMode.telemetry.addData("Servo1 Position", aimer1.getPosition());
        opMode.telemetry.addData("Servo2 Position", aimer2.getPosition());
        opMode.telemetry.addData("Encoder A", encoderA.getState());
        opMode.telemetry.addData("Encoder B", encoderB.getState());
    }

    public void stop() {
        turret.setPower(0);
        counterRoller.setVelocity(0);
    }


}
