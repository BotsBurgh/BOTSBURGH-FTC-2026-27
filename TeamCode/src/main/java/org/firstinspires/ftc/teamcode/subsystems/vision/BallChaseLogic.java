package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Hive Vision v1.1 library (copied from ftc_ball_chase_lib/final/)
import org.firstinspires.ftc.teamcode.BallChaseController;
import org.firstinspires.ftc.teamcode.BallTracker;

/**
 * All ball-chasing logic in one place.
 * Wraps Hive Vision's BallTracker + BallChaseController (camera-only chase).
 *
 * Usage:
 *   chase = new BallChaseLogic(hardwareMap, null);
 *   chase.startPollen();        // or startRed / startBlue / startAlliance / startAll
 *   chase.update();             // once per loop while active
 *   chase.abort();              // stop
 *
 * NOTE: while a chase is active this class drives the motors directly,
 * so don't also call Drivetrain.drive() at the same time.
 */
public class BallChaseLogic {

    // Class ids from labels.txt
    public static final int POLLEN = 0;   // yellow_pollen
    public static final int RED = 1;      // red_nectar
    public static final int BLUE = 2;     // blue_nectar

    private final BallTracker tracker;
    private final BallChaseController controller;

    private boolean active = false;

    /** @param intakeOrNull intake motor to run during pickup, or null */
    public BallChaseLogic(HardwareMap hardwareMap, DcMotor intakeOrNull) {

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        DcMotor lf = hardwareMap.get(DcMotor.class, "lf");
        DcMotor rf = hardwareMap.get(DcMotor.class, "rf");
        DcMotor lb = hardwareMap.get(DcMotor.class, "lr");
        DcMotor rb = hardwareMap.get(DcMotor.class, "rr");
        // Directions are NOT set here: Drivetrain already sets them
        // (left REVERSE / right FORWARD). configureMecanumDirections() uses the
        // opposite convention and would flip your robot.

        tracker = new BallTracker(limelight);
        controller = new BallChaseController(tracker, lf, rf, lb, rb, intakeOrNull);
        controller.setMaxPickups(0);   // 0 = keep going until aborted / field empty
    }

    // ---------- start functions (one per target) ----------

    public void startPollen() {
        tracker.setAllowedClasses(POLLEN);
        begin();
    }

    public void startRed() {
        tracker.setAllowedClasses(RED);
        begin();
    }

    public void startBlue() {
        tracker.setAllowedClasses(BLUE);
        begin();
    }

    /** Red + blue, ignores yellow. */
    public void startAlliance() {
        tracker.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
        begin();
    }

    /** Everything: pollen + red + blue. */
    public void startAll() {
        tracker.setAllowedClasses(POLLEN, RED, BLUE);
        begin();
    }

    private void begin() {
        controller.start();   // resets pickups + tracker lock
        active = true;
    }

    // ---------- loop / control ----------

    /** Call once per loop while active(). */
    public void update() {
        if (!active) {
            return;
        }
        controller.update();
        if (controller.isDone()) {
            controller.abort();
            active = false;
        }
    }

    /** Stop chasing and zero the motors. Safe to call any time. */
    public void abort() {
        controller.abort();
        active = false;
    }

    public void setMaxPickups(int n) {
        controller.setMaxPickups(n);
    }

    // ---------- status ----------

    public boolean isActive() {
        return active;
    }

    public int getPickups() {
        return controller.getPickups();
    }

    public int getFailedPickups() {
        return controller.getFailedPickups();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Chase active", active);
        controller.addTelemetry(telemetry);
    }
}