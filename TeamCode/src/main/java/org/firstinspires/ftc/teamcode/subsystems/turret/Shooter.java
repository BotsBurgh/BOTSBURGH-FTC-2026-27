package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.turret.*;

public class Shooter {

    private final DcMotorEx shooter;
    private final DcMotorEx transfer;

    // Inventory BEFORE shoot was pressed
    private int pollen = 0;
    private int nectar = 0;

    public Shooter(HardwareMap hardwareMap) {

        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shooter"
        );

        transfer = hardwareMap.get(
                DcMotorEx.class,
                "transfer"
        );

        shooter.setMode(
                DcMotorEx.RunMode.RUN_USING_ENCODER
        );
    }


    // ==========================================
    // INVENTORY
    // ==========================================

    public void setPollen(int amount) {
        pollen = amount;
    }

    public void setNectar(int amount) {
        nectar = amount;
    }

    public int getPollen() {
        return pollen;
    }

    public int getNectar() {
        return nectar;
    }


    // ==========================================
    // SHOOT
    // ==========================================

    public void shoot(double distance) {

        // Save what we had when SHOOT was pressed
        int pollenShot = pollen;
        int nectarShot = nectar;

        // Immediately clear inventory
        pollen = 0;
        nectar = 0;

        // Calculate shooter velocity
        double velocity =
                getVelocityFromDistance(distance);

        // Turn shooter ON
        shooter.setVelocity(velocity);

        // Turn transfer ON
        transfer.setPower(
                ShootingConstants.TRANSFER_POWER
        );
    }


    // ==========================================
    // STOP
    // ==========================================

    public void stop() {

        shooter.setVelocity(0);

        transfer.setPower(0);
    }


    // ==========================================
    // DISTANCE → SHOOTER VELOCITY
    // ==========================================

    private double getVelocityFromDistance(
            double distance) {

        distance = Math.max(
                ShootingConstants.MIN_DISTANCE,
                Math.min(
                        distance,
                        ShootingConstants.MAX_DISTANCE
                )
        );

        double ratio =
                (distance - ShootingConstants.MIN_DISTANCE)
                        /
                        (ShootingConstants.MAX_DISTANCE
                                - ShootingConstants.MIN_DISTANCE);

        return ShootingConstants.MIN_DISTANCE_VELOCITY
                +
                ratio *
                        (ShootingConstants.MAX_DISTANCE_VELOCITY
                                - ShootingConstants.MIN_DISTANCE_VELOCITY);
    }


    public double getShooterVelocity() {
        return shooter.getVelocity();
    }
}