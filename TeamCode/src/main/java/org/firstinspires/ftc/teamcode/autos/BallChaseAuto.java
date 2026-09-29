package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// Hive Vision v1.1 library. Copy the files from ftc_ball_chase_lib/final/
// (BallTracker, BallChaseController, HiveConfig, BallMath, etc.) into
// org.firstinspires.ftc.teamcode (they declare that package).
import org.firstinspires.ftc.teamcode.subsystems.vision.*;

@Autonomous(name = "Ball Chase (No Odometry)", group = "Vision")
public class BallChaseAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        // --- Limelight ---
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);   // Neural Detector pipeline w/ v1.1 SSD model
        limelight.start();

        // --- Drive motors (your config names; docs call the rear ones lb/rb) ---
        DcMotor lf = hardwareMap.get(DcMotor.class, "lf");
        DcMotor rf = hardwareMap.get(DcMotor.class, "rf");
        DcMotor lb = hardwareMap.get(DcMotor.class, "lr");
        DcMotor rb = hardwareMap.get(DcMotor.class, "rr");
        // Your robot's directions (same as Drivetrain). Don't use
        // configureMecanumDirections(): it uses the opposite convention.
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // --- Perception: only chase alliance balls (red + blue, ignore yellow) ---
        BallTracker tracker = new BallTracker(limelight);
        tracker.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
        // tracker.requireConfirmation(4);   // optional: same ball for 4 frames

        // --- Chase controller. Intake is null until you pass it a DcMotor ---
        BallChaseController chase =
                new BallChaseController(tracker, lf, rf, lb, rb, null);
        chase.setMaxPickups(3);

        // Optional: only count a pickup if the laser sensor sees the ball
        // BallSensor ballSensor = new BallSensor(hardwareMap);
        // chase.setPickupConfirmer(() -> ballSensor.hasBall());
        // (and call ballSensor.update() inside the loop below)

        telemetry.addLine("Ready. Press start.");
        telemetry.update();

        waitForStart();

        chase.start();

        while (opModeIsActive() && !chase.isDone()) {
            chase.update();                 // once per loop
            chase.addTelemetry(telemetry);
            telemetry.update();
        }

        chase.abort();                      // zeroes motors, safe to always call
        limelight.stop();
    }
}