
package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo and Encoder Test")
public class ServoEncoderTest extends OpMode {

    private Servo servo1;
    private Servo servo2;
    private DcMotorEx throughBoreEncoder;

    private TelemetryManager panels;

    private double servo1Position = 0.5;
    private double servo2Position = 0.5;

    private int encoderZero;

    private boolean previousA;
    private boolean previousB;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        // Configure the Through Bore Encoder's hub port as a motor port.
        // The encoder must be plugged into the matching encoder port.
        throughBoreEncoder =
                hardwareMap.get(DcMotorEx.class, "throughBoreEncoder");

        throughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo1.setPosition(servo1Position);
        servo2.setPosition(servo2Position);

        encoderZero = throughBoreEncoder.getCurrentPosition();

        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("Servo and Through Bore Encoder Test Ready");
        telemetry.addLine("D-pad adjusts servo positions");
        telemetry.addLine("A resets encoder reference");
        telemetry.addLine("B resets both servo commands to 0.5");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Adjust servo 1
        if (gamepad1.dpad_left) {
            servo1Position -= 0.005;
        }
        if (gamepad1.dpad_right) {
            servo1Position += 0.005;
        }

        // Adjust servo 2
        if (gamepad1.dpad_down) {
            servo2Position -= 0.005;
        }
        if (gamepad1.dpad_up) {
            servo2Position += 0.005;
        }

        servo1Position = Math.max(0.0, Math.min(1.0, servo1Position));
        servo2Position = Math.max(0.0, Math.min(1.0, servo2Position));

        servo1.setPosition(servo1Position);
        servo2.setPosition(servo2Position);

        // Read the Through Bore Encoder
        int ticks = throughBoreEncoder.getCurrentPosition();
        double velocity = throughBoreEncoder.getVelocity();

        // Reset encoder reference when A is pressed
        if (gamepad1.a && !previousA) {
            encoderZero = ticks;
        }
        previousA = gamepad1.a;

        // Reset commanded servo positions when B is pressed
        if (gamepad1.b && !previousB) {
            servo1Position = 0.5;
            servo2Position = 0.5;
        }
        previousB = gamepad1.b;

        int relativeTicks = ticks - encoderZero;

        // Driver Station telemetry
        telemetry.addLine("=== SERVO 1 ===");
        telemetry.addData("Commanded Position", servo1Position);

        telemetry.addLine("=== SERVO 2 ===");
        telemetry.addData("Commanded Position", servo2Position);

        telemetry.addLine("=== REV THROUGH BORE ENCODER ===");
        telemetry.addData("Raw Ticks", ticks);
        telemetry.addData("Ticks From Reference", relativeTicks);
        telemetry.addData("Absolute Ticks From Reference",
                Math.abs(relativeTicks));
        telemetry.addData("Velocity (ticks/sec)", velocity);

        telemetry.update();

        // Panels telemetry
        panels.debug("Servo 1 Position", servo1Position);
        panels.debug("Servo 2 Position", servo2Position);

        panels.debug("Through Bore Raw Ticks", ticks);
        panels.debug("Through Bore Relative Ticks", relativeTicks);
        panels.debug("Through Bore Absolute Ticks",
                Math.abs(relativeTicks));
        panels.debug("Through Bore Velocity", velocity);

        panels.update(telemetry);
    }

    @Override
    public void stop() {
        // Leave servos at their last commanded positions.
    }
}
