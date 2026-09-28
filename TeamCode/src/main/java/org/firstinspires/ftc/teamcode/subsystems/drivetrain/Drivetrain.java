package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    private final DcMotor lf;
    private final DcMotor lr;
    private final DcMotor rf;
    private final DcMotor rr;

    public Drivetrain(HardwareMap hardwareMap) {

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

    /** Full mecanum. forward +, strafe + = right, turn + = clockwise (right). */
    public void drive(double forward, double strafe, double turn) {

        double lfP = forward + strafe + turn;
        double lrP = forward - strafe + turn;
        double rfP = forward - strafe - turn;
        double rrP = forward + strafe - turn;

        // Scale all four down together so ratios (and direction) are kept
        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(lfP), Math.abs(lrP)),
                        Math.max(Math.abs(rfP), Math.abs(rrP))));

        lf.setPower(lfP / max);
        lr.setPower(lrP / max);
        rf.setPower(rfP / max);
        rr.setPower(rrP / max);
    }

    /** Kept for the vision follow OpMode (no strafing). */
    public void drive(double forward, double turn) {
        drive(forward, 0, turn);
    }

    public void stop() {
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }
}