/*
 * BallChaseController - no-odometry ball chase, packaged as a reusable tool
 * instead of an OpMode (SEARCHING -> CHASING -> COASTING -> PICKUP -> DONE).
 * Class docs moved to MODULES.md (#ballchasecontroller).
 */
package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveConfig;

public class BallChaseController {

    public enum State { IDLE, SEARCHING, CHASING, COASTING, PICKUP, DONE }

    /* All tunables live in ONE file: HiveConfig. This class reads them at point
     * of use, so editing HiveConfig (or a config system driving it at runtime)
     * takes effect immediately - nothing is copied here at class load. */

    private final BallTracker tracker;
    private final DcMotor lf, rf, lb, rb;
    private final DcMotor intake;                         // may be null

    private State state = State.IDLE;
    private int pickups = 0;
    private int failedPickups = 0;                        // dwell elapsed but unconfirmed within the confirm window
    private int maxPickups = 0;                           // 0 = unlimited
    private PickupConfirmer pickConfirm = null;

    private final ElapsedTime lastSeen = new ElapsedTime();
    private final ElapsedTime pickupTimer = new ElapsedTime();
    private double lastSeenDist = Double.MAX_VALUE;

    private final ElapsedTime searchClock = new ElapsedTime();  // sweep-cap bookkeeping
    private double searchSwept = 0;                            // dead-reckoned rotation while searching
    private boolean searchingWas = false;                      // was the LAST drive a search turn?

    public BallChaseController(BallTracker tracker, DcMotor lf, DcMotor rf,
                               DcMotor lb, DcMotor rb, DcMotor intakeOrNull) {
        this.tracker = tracker;
        this.lf = lf; this.rf = rf; this.lb = lb; this.rb = rb;
        this.intake = intakeOrNull;
    }

    /** Standard symmetric mecanum directions (LF F, LB F, RF R, RB R). Call once
     *  in your OpMode init unless a flipped gearbox changes the wiring. */
    public static void configureMecanumDirections(DcMotor lf, DcMotor rf,
                                                  DcMotor lb, DcMotor rb) {
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /* ---------------- public API ---------------- */

    public void setMaxPickups(int n) { maxPickups = n; }
    public int getPickups()           { return pickups; }
    public int getFailedPickups()     { return failedPickups; }
    public State getState()           { return state; }
    public boolean isDone()           { return state == State.DONE; }
    public boolean isActive()         { return state != State.IDLE && state != State.DONE; }

    /** Set a sensor check for a real ball entering the intake (color sensor,
     *  beam break, or current spike). Without one, pickups are counted after
     *  every dwell. */
    public void setPickupConfirmer(PickupConfirmer c) { pickConfirm = c; }

    /** Begin a fresh hunt: reset pickups and the target lock. */
    public void start() {
        pickups = 0;
        failedPickups = 0;
        tracker.reset();
        searchSwept = 0;
        searchingWas = false;
        state = State.SEARCHING;
    }

    /** Stop everything: motors zeroed, intake off, back to IDLE. */
    public void abort() {
        drive(0, 0, 0);
        setIntake(0);
        tracker.reset();
        state = State.IDLE;
    }

    /** Drive the state machine one step. Call every loop while active. */
    public void update() {
        if (!isActive()) return;

        if (state == State.PICKUP) {
            doPickup();
            return;
        }

        BallTracker.Sighting s = tracker.update();

        if (s == null) {
            noTarget();
            return;
        }

        searchSwept = 0;        // something is visible: a fresh sweep starts if it vanishes again
        searchingWas = false;

        double tx = s.txDeg;

        if (!s.predicted) {
            // trusted fresh detection: refresh the "last seen" close-distance bookkeeping
            lastSeen.reset();
            lastSeenDist = s.distIn;
        } else {
            // ball is momentarily missing but the lock is still fresh: mirror the
            // real-loss handling for close balls (coast, else pickup); otherwise keep
            // facing the last-known angle without advancing or resetting lastSeen.
            boolean wasClose = lastSeenDist <= HiveConfig.COAST_MAX_DIST;
            if (wasClose) {
                if (lastSeen.milliseconds() < HiveConfig.COAST_MS) {
                    state = State.COASTING;
                    setIntake(HiveConfig.INTAKE_POWER);
                    drive(HiveConfig.COAST_POWER, 0, 0);
                    return;
                }
                state = State.PICKUP;
                pickupTimer.reset();
                drive(0, 0, 0);
                return;
            }
            // A predicted turn uses the last-known angle, which goes STALE while
            // the ball is missing - after PREDICT_TURN_MS of no real sighting,
            // hold position instead of creeping toward an old angle.
            if (lastSeen.milliseconds() > HiveConfig.PREDICT_TURN_MS) {
                drive(0, 0, 0);
                return;
            }
            double turn = Range.clip(tx * HiveConfig.TURN_KP, -HiveConfig.MAX_TURN, HiveConfig.MAX_TURN);
            if (Math.abs(turn) < HiveConfig.MIN_TURN) turn = Math.copySign(HiveConfig.MIN_TURN, tx);
            drive(0, 0, turn);
            return;
        }

        // ---- real target: chase it ----
        boolean aimed = Math.abs(tx) < HiveConfig.AIM_TOL_DEG;

        if (s.distIn <= HiveConfig.STOP_DIST && aimed) {
            state = State.PICKUP;
            pickupTimer.reset();
            drive(0, 0, 0);
            return;
        }

        // !!aimed guarantees |turn| = |tx|*TURN_KP >= AIM_TOL_DEG*TURN_KP way
        // above the MIN_TURN friction floor, so a MIN_TURN clamp here is dead
        // code - at these gains the P term already clears the floor.
        double turn = Range.clip(tx * HiveConfig.TURN_KP, -HiveConfig.MAX_TURN, HiveConfig.MAX_TURN);

        double fwd = 0.0;
        if (s.distIn > HiveConfig.STOP_DIST && Math.abs(tx) < HiveConfig.DRIVE_MIN_TX) {
            fwd = Range.clip((s.distIn - HiveConfig.STOP_DIST) * HiveConfig.DRIVE_KP,
                    HiveConfig.MIN_FWD, HiveConfig.MAX_FWD);
        }

        state = State.CHASING;
        setIntake(HiveConfig.INTAKE_POWER);
        drive(fwd, 0, turn);
    }

    public void addTelemetry(Telemetry t) {
        BallTracker.Sighting s = tracker.getLast();
        t.addData("chase", "%s  pickups=%d/%s failed=%d", state, pickups,
                maxPickups > 0 ? Integer.toString(maxPickups) : "unlimited",
                failedPickups);
        if (s != null) {
            t.addData("chase target", "tx=%+.1f ty=%+.1f dist=%.1f in cls=%d conf=%.2f%s",
                    s.txDeg, s.tyDeg, s.distIn, s.classId, s.confidence,
                    s.predicted ? " (predicted)" : "");
        } else {
            t.addData("chase target", "none");
        }
    }

    /* ---------------- states ---------------- */

    private void doPickup() {
        drive(0, 0, 0);
        setIntake(HiveConfig.INTAKE_POWER);
        if (pickupTimer.milliseconds() < HiveConfig.PICKUP_DWELL_MS) return;

        boolean confirmed = pickConfirm == null || pickConfirm.isBallInIntake();
        if (!confirmed) {
            // Missed grab (ball bounced off / rolled away): keep the intake
            // on, but give up waiting past the confirm window instead of
            // crediting a pickup the robot never made. A window <= 0 is a
            // ONE-SHOT check at the end of the dwell - never wait forever.
            if (HiveConfig.PICKUP_CONFIRM_MS <= 0
                    || pickupTimer.milliseconds() >= HiveConfig.PICKUP_DWELL_MS + HiveConfig.PICKUP_CONFIRM_MS) {
                failedPickups++;
                tracker.reset();
                state = State.SEARCHING;
            }
            return;
        }

        pickups++;
        tracker.reset();                 // drop any lock so the next ball can be picked
        if (maxPickups > 0 && pickups >= maxPickups) {
            setIntake(0);
            state = State.DONE;
        } else {
            state = State.SEARCHING;
        }
    }

    private void noTarget() {
        boolean wasClose = (state == State.CHASING || state == State.COASTING)
                && lastSeenDist <= HiveConfig.COAST_MAX_DIST;

        if (wasClose) {
            if (lastSeen.milliseconds() < HiveConfig.COAST_MS) {
                state = State.COASTING;
                setIntake(HiveConfig.INTAKE_POWER);
                drive(HiveConfig.COAST_POWER, 0, 0);   // ball went under the intake: keep going straight
            } else {
                state = State.PICKUP;
                pickupTimer.reset();
                drive(0, 0, 0);
            }
            return;
        }

        state = State.SEARCHING;
        searchTurn();
    }

    /** Rotate in place looking for a ball, but only inside the dead-reckoned
     *  SEARCH_SWEEP_DEG budget: an empty field makes the hunt end cleanly
     *  instead of spinning until the opmode is stopped. Only time actually
     *  spent turning counts (a pickup dwell or chase gap doesn't consume sweep). */
    private void searchTurn() {
        if (searchingWas) {
            double dt = Math.min(searchClock.seconds(), 0.1);
            searchSwept += HiveConfig.SEARCH_TURN * HiveConfig.TURN_RATE_RAD_PER_POWER_SEC * dt;
            if (searchSwept >= Math.toRadians(HiveConfig.SEARCH_SWEEP_DEG)) {
                setIntake(0);
                drive(0, 0, 0);
                state = State.DONE;
                return;
            }
        }
        searchClock.reset();
        searchingWas = true;
        setIntake(0);
        drive(0, 0, HiveConfig.SEARCH_TURN);
    }

    /* ---------------- helpers ---------------- */

    /** Mecanum: fwd + = forward, strafe + = right, turn + = clockwise. Normalized. */
    private void drive(double fwd, double strafe, double turn) {
        double pLf = fwd + strafe + turn;
        double pRf = fwd - strafe - turn;
        double pLb = fwd - strafe + turn;
        double pRb = fwd + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(pLf), Math.abs(pRf)),
                        Math.max(Math.abs(pLb), Math.abs(pRb))));

        lf.setPower(pLf / max);
        rf.setPower(pRf / max);
        lb.setPower(pLb / max);
        rb.setPower(pRb / max);
    }

    private void setIntake(double power) {
        if (intake != null) intake.setPower(power);
    }
}