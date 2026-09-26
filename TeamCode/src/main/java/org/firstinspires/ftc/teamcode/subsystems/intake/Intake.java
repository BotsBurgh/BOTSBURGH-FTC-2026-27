package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intake;

    public Intake(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    // Set any power from -1.0 to 1.0
    public void setPower(double power) {
        intake.setPower(power);
    }

    // Full speed forward
    public void forward() {
        intake.setPower(1.0);
    }

    // Full speed reverse
    public void reverse() {
        intake.setPower(-1.0);
    }

    // Stop intake
    public void stop() {
        intake.setPower(0);
    }
}