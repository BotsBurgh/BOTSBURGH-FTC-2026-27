package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.List;

/**
 * TunerTestingTelOp - High-performance OpMode for testing and tuning shooter PIDF parameters.
 * Features ultra-fast loop rates, Lynx manual bulk caching, smooth Hz tracking, and derivative analytics.
 */
@TeleOp(name = "TunerTesting", group = "Tuning")
public class TunerTestingTelOp extends OpMode {

    // --- Subsystems & Hardware ---
    private Robot robot;
    private LynxModule[] lynxModules;

    // --- Telemetry ---
    private TelemetryManager panels;
    private static final long TELEMETRY_INTERVAL_NANO = 20_000_000L; // 50 FPS (20ms)
    private long lastTelemetryNanoTime = 0;

    // --- Smooth Hz Tracking ---
    private int framesInWindow = 0;
    private double smoothedHz = 0.0;
    private long lastLoopNanoTime = 0;

    // --- Derivative Calculations ---
    private double lastError = 0.0;
    private double lastActualVel = 0.0;
    private double errorDerivative = 0.0;
    private double velocityAcceleration = 0.0;

    @Override
    public void init() {
        // Optimize USB transmission interval
        telemetry.setMsTransmissionInterval(11);

        // Configure REV Hub Lynx Modules for Manual Bulk Caching
        List<LynxModule> moduleList = hardwareMap.getAll(LynxModule.class);
        lynxModules = moduleList.toArray(new LynxModule[0]);
        for (LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize Subsystems & Telemetry
        robot = new Robot(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // Default state
        ShooterConfig.ENABLED = false;

        long now = System.nanoTime();
        lastTelemetryNanoTime = now;
        lastLoopNanoTime = now;
    }

    @Override
    public void loop() {
        // 1. Clear Lynx Bulk Cache once per loop cycle
        clearBulkCache();

        // 2. Track loop metrics
        framesInWindow++;
        long now = System.nanoTime();

        // Calculate time delta for loop and derivative
        double dtSeconds = (now - lastLoopNanoTime) / 1e9;
        lastLoopNanoTime = now;

        // 3. Gamepad controls
        if (gamepad1.a) {
            ShooterConfig.ENABLED = false;
        }

        // 4. Subsystem update
        robot.update();

        // 5. Derivative calculations (Error Derivative dE/dt and Acceleration dV/dt)
        calculateDerivatives(dtSeconds);

        // 6. Rate-limited telemetry update with smooth Hz calculation
        if (now - lastTelemetryNanoTime >= TELEMETRY_INTERVAL_NANO) {
            double windowDurationSec = (now - lastTelemetryNanoTime) / 1e9;
            double currentWindowHz = framesInWindow / windowDurationSec;
            framesInWindow = 0;
            lastTelemetryNanoTime = now;

            // Exponential Moving Average filter for smooth Hz display
            if (smoothedHz == 0.0) {
                smoothedHz = currentWindowHz;
            } else {
                smoothedHz = (smoothedHz * 0.85) + (currentWindowHz * 0.15);
            }

            renderTelemetry((int) Math.round(smoothedHz));
        }
    }

    /**
     * Calculates error derivative (dError/dt) and motor acceleration (dVel/dt).
     */
    private void calculateDerivatives(double dtSeconds) {
        if (dtSeconds > 0.0000001) {
            double targetVel = robot.shooter.getTurretTargetVelocity();
            double actualVel = robot.shooter.getTurretVelocity();
            double currentError = targetVel - actualVel;

            double rawErrorDeriv = (currentError - lastError) / dtSeconds;
            double rawVelAccel = (actualVel - lastActualVel) / dtSeconds;

            // Filter derivative noise
            errorDerivative = (errorDerivative * 0.8) + (rawErrorDeriv * 0.2);
            velocityAcceleration = (velocityAcceleration * 0.8) + (rawVelAccel * 0.2);

            lastError = currentError;
            lastActualVel = actualVel;
        }
    }

    /**
     * Clears Lynx Bulk Data Caches across all REV Hubs.
     */
    private void clearBulkCache() {
        for (LynxModule module : lynxModules) {
            module.clearBulkCache();
        }
    }

    /**
     * Formats and updates telemetry panel debug data.
     */
    private void renderTelemetry(int currentHz) {
        double targetVel = robot.shooter.getTurretTargetVelocity();
        double actualVel = robot.shooter.getTurretVelocity();

        double rollerTargetVel = ShooterConfig.ENABLED
                ? ShooterConfig.TARGET_VELOCITY * ShooterConfig.ROLLER_TURRET_RATIO
                : 0.0;

        double rollerActualVel = robot.shooter.getCounterRollerVelocity();

        panels.debug("Hz", currentHz);
        panels.debug("Enabled", ShooterConfig.ENABLED);
        panels.debug("Target Velocity", targetVel);
        panels.debug("Actual Velocity", actualVel);
        panels.debug("Velocity Error", targetVel - actualVel);
        panels.debug("Error Derivative (dE/dt)", errorDerivative);
        panels.debug("Turret Acceleration (dV/dt)", velocityAcceleration);
        panels.debug("Turret Ticks", robot.shooter.getTurretTicks());

        panels.debug("Counter Roller Target", rollerTargetVel);
        panels.debug("Counter Roller Actual", rollerActualVel);

        panels.debug("P", ShooterConfig.TURRET_P);
        panels.debug("I", ShooterConfig.TURRET_I);
        panels.debug("D", ShooterConfig.TURRET_D);
        panels.debug("F", ShooterConfig.TURRET_F);
        panels.debug("Roller Ratio", ShooterConfig.ROLLER_TURRET_RATIO);

        panels.update(telemetry);
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
