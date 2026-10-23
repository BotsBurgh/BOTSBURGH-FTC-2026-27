package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;

/**
 * Subsystem representing the Turret and Counter-Roller Mechanism.
 * Uses hardware write caching and bulk-cached velocity estimation for high-frequency loop performance.
 */
public class Shooter {

    // Hardware Motors
    private final DcMotorEx turret;
    private final DcMotorEx counterRoller;

    // Hardware State Cache (Prevents redundant USB writes)
    private double cachedP = Double.NaN;
    private double cachedI = Double.NaN;
    private double cachedD = Double.NaN;
    private double cachedF = Double.NaN;

    private double cachedTurretTargetVel = Double.NaN;
    private double cachedRollerTargetVel = Double.NaN;
    private boolean isShooterActive = false;

    // Zero-Latency Velocity Estimator (Uses Lynx Bulk Data Cache)
    private int lastTurretTicks = 0;
    private int lastRollerTicks = 0;
    private int currentTurretTicks = 0;
    private long lastVelUpdateNano = 0;

    private double turretVelocity = 0.0;
    private double counterRollerVelocity = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        counterRoller = hardwareMap.get(DcMotorEx.class, "counterRoller");

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counterRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Main update loop for the shooter subsystem. Should be called once per OpMode loop cycle.
     */
    public void runTurret() {
        updateVelocityEstimates();
        syncPIDFCoefficients();
        updateMotorOutputs();
    }

    /**
     * Calculates velocity from bulk-cached encoder ticks in 0 microseconds (no USB commands).
     */
    private void updateVelocityEstimates() {
        long now = System.nanoTime();
        currentTurretTicks = turret.getCurrentPosition();
        int currentRollerTicks = counterRoller.getCurrentPosition();

        if (lastVelUpdateNano != 0) {
            double dt = (now - lastVelUpdateNano) / 1e9;
            if (dt > 0.0001) {
                double rawTurretVel = (currentTurretTicks - lastTurretTicks) / dt;
                double rawRollerVel = (currentRollerTicks - lastRollerTicks) / dt;

                // Smooth raw velocity readings using a low-pass filter
                turretVelocity = (turretVelocity * 0.75) + (rawTurretVel * 0.25);
                counterRollerVelocity = (counterRollerVelocity * 0.75) + (rawRollerVel * 0.25);

                lastTurretTicks = currentTurretTicks;
                lastRollerTicks = currentRollerTicks;
                lastVelUpdateNano = now;
            }
        } else {
            lastTurretTicks = currentTurretTicks;
            lastRollerTicks = currentRollerTicks;
            lastVelUpdateNano = now;
        }
    }

    /**
     * Updates PIDF coefficients on hardware only if they changed in configuration.
     */
    private void syncPIDFCoefficients() {
        if (ShooterConfig.TURRET_P != cachedP ||
                ShooterConfig.TURRET_I != cachedI ||
                ShooterConfig.TURRET_D != cachedD ||
                ShooterConfig.TURRET_F != cachedF) {

            cachedP = ShooterConfig.TURRET_P;
            cachedI = ShooterConfig.TURRET_I;
            cachedD = ShooterConfig.TURRET_D;
            cachedF = ShooterConfig.TURRET_F;

            turret.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(cachedP, cachedI, cachedD, cachedF)
            );
        }
    }

    /**
     * Updates motor velocity targets or powers only when targets or enabled states change.
     */
    private void updateMotorOutputs() {
        if (ShooterConfig.ENABLED) {
            double targetVelocity = ShooterConfig.TARGET_VELOCITY;
            double rollerTargetVelocity = targetVelocity * ShooterConfig.ROLLER_TURRET_RATIO;

            if (!isShooterActive || cachedTurretTargetVel != targetVelocity) {
                turret.setVelocity(targetVelocity);
                cachedTurretTargetVel = targetVelocity;
            }

            if (!isShooterActive || cachedRollerTargetVel != rollerTargetVelocity) {
                counterRoller.setVelocity(rollerTargetVelocity);
                cachedRollerTargetVel = rollerTargetVelocity;
            }

            isShooterActive = true;
        } else {
            if (isShooterActive) {
                turret.setPower(0);
                counterRoller.setPower(0);
                cachedTurretTargetVel = Double.NaN;
                cachedRollerTargetVel = Double.NaN;
                isShooterActive = false;
            }
        }
    }

    // --- Getters ---

    public double getTurretTargetVelocity() {
        return ShooterConfig.ENABLED ? ShooterConfig.TARGET_VELOCITY : 0.0;
    }

    public double getTurretVelocity() {
        return turretVelocity;
    }

    public double getCounterRollerVelocity() {
        return counterRollerVelocity;
    }

    public int getTurretTicks() {
        return currentTurretTicks;
    }

    /**
     * Stops all motors and resets motor state cache.
     */
    public void stop() {
        turret.setPower(0);
        counterRoller.setPower(0);
        isShooterActive = false;
        cachedTurretTargetVel = Double.NaN;
        cachedRollerTargetVel = Double.NaN;
    }
}
