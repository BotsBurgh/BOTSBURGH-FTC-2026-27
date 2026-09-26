package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drivetrain {

    private final DcMotor lf;
    private final DcMotor lr;
    private final DcMotor rf;
    private final DcMotor rr;

    public drivetrain(HardwareMap hardwareMap) {

        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);

        rf.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double forward, double turn) {

        double leftPower = forward + turn;
        double rightPower = forward - turn;

        leftPower = clip(leftPower);
        rightPower = clip(rightPower);

        lf.setPower(leftPower);
        lr.setPower(leftPower);

        rf.setPower(rightPower);
        rr.setPower(rightPower);
    }

    public void stop() {

        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }

    private double clip(double power) {

        if (power > 1) {
            return 1;
        }

        if (power < -1) {
            return -1;
        }

        return power;
    }
}