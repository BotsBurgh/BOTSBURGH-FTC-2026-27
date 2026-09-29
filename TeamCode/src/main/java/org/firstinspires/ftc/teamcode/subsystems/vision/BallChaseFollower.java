///*
// * BallChaseFollower - hybrid ball collection for AUTO: Pedro plans the
// * approach, the camera finishes (SCAN/TRAVEL/TURN/CHASE/PICKUP/DONE).
// * Class docs moved to MODULES.md (#ballchasefollower).
// */
//package org.firstinspires.ftc.teamcode.subsystems.vision;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.HiveConfig;
//
//import java.util.ArrayList;
//import java.util.List;
//import java.util.Set;
//
//public class BallChaseFollower {
//
//    public enum State { IDLE, SCAN, TRAVEL, TURN, CHASE, PICKUP, DONE }
//
//    /* All tunables live in ONE file: HiveConfig. This class reads them at point
//     * of use, so editing HiveConfig (or a config system driving it at runtime)
//     * takes effect immediately - nothing is copied here at class load. */
//
//    private static class Ball {
//        double x, y;
//        int n = 1;
//        Ball(double x, double y) { this.x = x; this.y = y; }
//    }
//
//    private final Follower follower;
//    private final BallTracker tracker;
//    private final DcMotor intake;                         // may be null
//
//    private State state = State.IDLE;
//    private State afterTurn = State.SCAN;
//    private boolean teleop = false;
//
//    private List<Ball> memory = new ArrayList<>();
//    private final List<Ball> samples = new ArrayList<>();
//    private Ball currentTarget = null;
//
//    private final ElapsedTime stateTimer = new ElapsedTime();
//    private final ElapsedTime lastSeen = new ElapsedTime();
//    private final ElapsedTime total = new ElapsedTime();
//    private double lastSeenDist = Double.MAX_VALUE;
//    private boolean chaseSawBall = false;
//    private boolean haveChaseBall = false;
//    private double lastChaseX, lastChaseY;
//    private double turnTarget = 0;
//
//    private int pickups = 0;
//    private int maxPickups = 0;                           // 0 = unlimited
//    private int failedPickups = 0;                        // dwell elapsed but unconfirmed within the confirm window
//    private PickupConfirmer pickConfirm = null;
//    private int searchSteps = 0;                          // consecutive EMPTY scans without finding a ball
//    private int failedApproaches = 0;                     // consecutive TRAVEL timeouts; NOT reset on re-find
//    private boolean budgetUserSet = false;                // explicit setTimeBudgetSec() beats the HiveConfig default
//    private double budgetSec = Double.MAX_VALUE;
//    private long lastSampleFrame = -1;                    // frame id the SCAN sampler last consumed
//
//    public BallChaseFollower(Follower follower, BallTracker tracker, DcMotor intakeOrNull) {
//        this.follower = follower;
//        this.tracker = tracker;
//        this.intake = intakeOrNull;
//    }
//
//    /** Convenience: build a default BallTracker over the Limelight for you. */
//    public BallChaseFollower(Follower follower, Limelight3A limelight, DcMotor intakeOrNull) {
//        this(follower, new BallTracker(limelight), intakeOrNull);
//    }
//
//    /* ---------------- public API ---------------- */
//
//    /** Which classes to chase; delegated straight to the tracker. */
//    public void setAllowedClasses(Set<Integer> classes) { tracker.setAllowedClasses(classes); }
//    public void setAllowedClasses(Integer... classes)     { tracker.setAllowedClasses(classes); }
//
//    public void setMaxPickups(int n)       { maxPickups = n; }
//    public void setTimeBudgetSec(double s) { budgetSec = s; budgetUserSet = true; }
//    public int getPickups()                { return pickups; }
//    public int getFailedPickups()          { return failedPickups; }
//
//    /** Set a sensor check for a real ball entering the intake (color sensor,
//     *  beam break, or current spike). Without one, pickups are counted after
//     *  every dwell. */
//    public void setPickupConfirmer(PickupConfirmer c) { pickConfirm = c; }
//    public State getState()                { return state; }
//    public boolean isDone()                { return state == State.DONE; }
//    public boolean isActive()              { return state != State.IDLE && state != State.DONE; }
//
//    public void start() {
//        memory.clear();
//        pickups = 0;
//        failedPickups = 0;
//        searchSteps = 0;
//        failedApproaches = 0;
//        currentTarget = null;
//        lastSampleFrame = -1;
//        if (!budgetUserSet) {
//            budgetSec = HiveConfig.HUNT_BUDGET_SEC > 0 ? HiveConfig.HUNT_BUDGET_SEC : Double.MAX_VALUE;
//        }
//        tracker.reset();
//        total.reset();
//        enter(State.SCAN);
//    }
//
//    public void abort() {
//        follower.breakFollowing();
//        follower.startTeleopDrive();
//        teleop = true;
//        setIntake(0);
//        drive(0, 0, 0);
//        state = State.IDLE;
//    }
//
//    public void update() {
//        if (!isActive()) return;
//        if (total.seconds() > budgetSec) { finish(); return; }
//
//        switch (state) {
//            case SCAN:    doScan();    break;
//            case TRAVEL:  doTravel();  break;
//            case TURN:    doTurn();    break;
//            case CHASE:   doChase();   break;
//            case PICKUP:  doPickup();  break;
//            default: break;
//        }
//    }
//
//    public void addTelemetry(Telemetry t) {
//        t.addData("hunt", "%s  pickups=%d/%s failed=%d  remembered=%d", state, pickups,
//                maxPickups > 0 ? Integer.toString(maxPickups) : "unlimited",
//                failedPickups, memory.size());
//        BallTracker.Sighting s = tracker.getLast();
//        if (s != null) {
//            t.addData("hunt target", "tx=%+.1f ty=%+.1f dist=%.1f in cls=%d conf=%.2f%s",
//                    s.txDeg, s.tyDeg, s.distIn, s.classId, s.confidence,
//                    s.predicted ? " (predicted)" : "");
//        }
//        for (int i = 0; i < memory.size() && i < 4; i++) {
//            Ball b = memory.get(i);
//            t.addData("  ball" + i, "(%.1f, %.1f) n=%d", b.x, b.y, b.n);
//        }
//    }
//
//    /* ---------------- states ---------------- */
//
//    private void doScan() {
//        drive(0, 0, 0);
//        setIntake(0);
//        double t = stateTimer.milliseconds();
//        if (t < HiveConfig.SETTLE_MS) return;
//
//        Pose pose = follower.getPose();
//        if (t < HiveConfig.SETTLE_MS + HiveConfig.SCAN_MS) {
//            sampleDetections(pose);
//            return;
//        }
//        finishScan(pose);
//    }
//
//    private void sampleDetections(Pose pose) {
//        // Only use frames captured after the stop (staleness in ms, both sides),
//        // and only GENUINELY NEW frames - re-polling the same camera frame must
//        // not double-count a ball into the scan.
//        if (tracker.getStalenessMs() > stateTimer.milliseconds() - HiveConfig.SETTLE_MS) return;
//        long f = tracker.getFrame();
//        if (f == lastSampleFrame) return;
//        lastSampleFrame = f;
//
//        List<BallTracker.RawDet> dets = tracker.getGatedDetections();
//        if (dets == null) return;
//
//        for (BallTracker.RawDet d : dets) {
//            double[] p = project(d.txDeg, d.tyDeg, pose);
//            if (p == null) continue;
//            if (Math.hypot(p[0] - pose.getX(), p[1] - pose.getY()) > HiveConfig.MAX_PLAN_RANGE) continue;
//            addSample(p[0], p[1]);
//        }
//    }
//
//    private void addSample(double x, double y) {
//        for (Ball b : samples) {
//            if (Math.hypot(b.x - x, b.y - y) < HiveConfig.MERGE_RADIUS) {
//                b.x = (b.x * b.n + x) / (b.n + 1);
//                b.y = (b.y * b.n + y) / (b.n + 1);
//                b.n++;
//                return;
//            }
//        }
//        samples.add(new Ball(x, y));
//    }
//
//    private void finishScan(Pose pose) {
//        // Keep out-of-view balls, drop ones that should be right in front of the camera.
//        List<Ball> next = new ArrayList<>();
//        for (Ball b : memory) {
//            if (!inView(b, pose)) next.add(b);
//        }
//        next.addAll(samples);
//        memory = next;
//
//        if (maxPickups > 0 && pickups >= maxPickups) { finish(); return; }
//
//        Ball target = nearest(memory, pose);
//        if (target == null) {
//            if (searchSteps >= HiveConfig.SEARCH_MAX_STEPS) { finish(); return; }
//            searchSteps++;
//            startTurn(pose.getHeading() + Math.toRadians(HiveConfig.SEARCH_STEP_DEG), State.SCAN);
//            return;
//        }
//
//        searchSteps = 0;
//        currentTarget = target;
//        double dx = target.x - pose.getX();
//        double dy = target.y - pose.getY();
//        double dist = Math.hypot(dx, dy);
//        double heading = Math.atan2(dy, dx);
//
//        if (dist > HiveConfig.FOLLOWER_APPROACH_DIST + 4.0) {
//            startTravel(pose, target, heading);
//        } else {
//            startTurn(heading, State.CHASE);   // already close: just face it
//        }
//    }
//
//    private void startTravel(Pose pose, Ball target, double heading) {
//        double ax = Range.clip(target.x - HiveConfig.FOLLOWER_APPROACH_DIST * Math.cos(heading),
//                HiveConfig.FIELD_MIN, HiveConfig.FIELD_MAX);
//        double ay = Range.clip(target.y - HiveConfig.FOLLOWER_APPROACH_DIST * Math.sin(heading),
//                HiveConfig.FIELD_MIN, HiveConfig.FIELD_MAX);
//
//        if (Math.hypot(ax - pose.getX(), ay - pose.getY()) < 3.0) {
//            startTurn(heading, State.CHASE);   // zero-length path guard
//            return;
//        }
//
//        // Shortest-way heading change, so we never spin across the +-pi seam.
//        double endHeading = pose.getHeading() + wrap(heading - pose.getHeading());
//
//        PathChain path = follower.pathBuilder()
//                .addPath(new BezierLine(pose, new Pose(ax, ay)))
//                .setLinearHeadingInterpolation(pose.getHeading(), endHeading)
//                .build();
//
//        follower.followPath(path, true);
//        teleop = false;
//        enter(State.TRAVEL);
//    }
//
//    private void doTravel() {
//        setIntake(0);
//        if (!follower.isBusy()) {
//            enter(State.CHASE);
//        } else if (stateTimer.milliseconds() > HiveConfig.TRAVEL_TIMEOUT_MS) {
//            follower.breakFollowing();
//            teleop = false;
//            searchSteps++;                 // a timed-out travel is also not a find
//            failedApproaches++;            // and one more failed approach - this is NOT reset on re-find
//            if (searchSteps >= HiveConfig.SEARCH_MAX_STEPS
//                    || failedApproaches >= HiveConfig.SEARCH_MAX_STEPS) { finish(); return; }
//            enter(State.SCAN);             // stuck or blocked: re-plan from wherever we are
//        }
//    }
//
//    private void startTurn(double targetHeading, State next) {
//        turnTarget = targetHeading;
//        afterTurn = next;
//        enter(State.TURN);
//    }
//
//    private void doTurn() {
//        setIntake(0);
//        double err = wrap(turnTarget - follower.getPose().getHeading());   // + = need CCW
//        if (Math.abs(err) < Math.toRadians(HiveConfig.TURN_TO_TOL_DEG)
//                || stateTimer.milliseconds() > HiveConfig.TURN_TIMEOUT_MS) {
//            drive(0, 0, 0);
//            enter(afterTurn);
//            return;
//        }
//        double cmd = Range.clip(err * HiveConfig.TURN_TO_KP, -HiveConfig.TURN_TO_MAX, HiveConfig.TURN_TO_MAX);
//        if (Math.abs(cmd) < HiveConfig.TURN_TO_MIN) cmd = Math.copySign(HiveConfig.TURN_TO_MIN, err);
//        double tol = Math.toRadians(HiveConfig.TURN_TO_TOL_DEG);
//        cmd *= Math.min(1.0, Math.abs(err) / (2.0 * tol));   // EASE into the 4 deg stop: no overshoot past it
//        drive(0, 0, cmd);
//    }
//
//    private void doChase() {
//        Pose pose = follower.getPose();
//        BallTracker.Sighting s = tracker.update();
//
//        if (s != null && !s.predicted) {
//            double tx = s.txDeg;
//            double ty = s.tyDeg;
//            double dist = s.distIn;
//
//            lastSeen.reset();
//            lastSeenDist = dist;
//            chaseSawBall = true;
//
//            double[] p = project(tx, ty, pose);
//            if (p != null) { lastChaseX = p[0]; lastChaseY = p[1]; haveChaseBall = true; }
//
//            boolean aimed = Math.abs(tx) < HiveConfig.AIM_TOL_DEG;
//            if (dist <= HiveConfig.STOP_DIST && aimed) {
//                drive(0, 0, 0);
//                enter(State.PICKUP);
//                return;
//            }
//
//            // !!aimed guarantees |turn| = |tx|*TURN_KP >= AIM_TOL_DEG*TURN_KP way
//            // above the MIN_TURN friction floor, so a MIN_TURN clamp here is dead
//            // code - at these gains the P term already clears the floor.
//            double turn = Range.clip(-tx * HiveConfig.TURN_KP, -HiveConfig.MAX_TURN, HiveConfig.MAX_TURN);
//
//            double fwd = 0.0;
//            if (dist > HiveConfig.STOP_DIST && Math.abs(tx) < HiveConfig.DRIVE_MIN_TX) {
//                fwd = Range.clip((dist - HiveConfig.STOP_DIST) * HiveConfig.DRIVE_KP,
//                        HiveConfig.MIN_FWD, HiveConfig.MAX_FWD);
//            }
//            setIntake(HiveConfig.INTAKE_POWER);
//            drive(fwd, 0, turn);
//            return;
//        }
//
//        if (s != null) {
//            // predicted: locked ball briefly missing (fresh lock) - mirror the
//            // real-loss handling for close balls (coast, else pickup); otherwise keep
//            // facing the last-known angle at reduced power without re-planning.
//            boolean wasClose = lastSeenDist <= HiveConfig.COAST_MAX_DIST;
//            if (wasClose) {
//                if (lastSeen.milliseconds() < HiveConfig.COAST_MS) {
//                    setIntake(HiveConfig.INTAKE_POWER);
//                    drive(HiveConfig.COAST_POWER, 0, 0);
//                    return;
//                }
//                drive(0, 0, 0);
//                enter(State.PICKUP);
//                return;
//            }
//            // A predicted turn uses the last-known angle, which goes STALE while
//            // the ball is missing - after PREDICT_TURN_MS of no real sighting,
//            // hold position instead of creeping toward an old angle.
//            if (lastSeen.milliseconds() > HiveConfig.PREDICT_TURN_MS) {
//                drive(0, 0, 0);
//                return;
//            }
//            double turn = Range.clip(-s.txDeg * HiveConfig.TURN_KP, -HiveConfig.MAX_TURN, HiveConfig.MAX_TURN);
//            if (Math.abs(turn) < HiveConfig.MIN_TURN) turn = Math.copySign(HiveConfig.MIN_TURN, -s.txDeg);
//            drive(0, 0, turn);
//            return;
//        }
//
//        // fully lost (lock dropped or never established)
//        if (chaseSawBall && lastSeenDist <= HiveConfig.COAST_MAX_DIST) {
//            if (lastSeen.milliseconds() < HiveConfig.COAST_MS) {
//                setIntake(HiveConfig.INTAKE_POWER);
//                drive(HiveConfig.COAST_POWER, 0, 0);   // ball went under the camera: keep going straight
//            } else {
//                drive(0, 0, 0);
//                enter(State.PICKUP);
//            }
//            return;
//        }
//
//        drive(0, 0, 0);
//        if (lastSeen.milliseconds() > HiveConfig.CHASE_LOST_MS) {
//            tracker.reset();                 // stale lock/candidate must not re-adopt the last ball
//            if (currentTarget != null) memory.remove(currentTarget);
//            enter(State.SCAN);
//        }
//    }
//
//    private void doPickup() {
//        drive(0, 0, 0);
//        setIntake(HiveConfig.INTAKE_POWER);
//        if (stateTimer.milliseconds() < HiveConfig.PICKUP_DWELL_MS) return;
//
//        boolean confirmed = pickConfirm == null || pickConfirm.isBallInIntake();
//        if (!confirmed) {
//            // Missed grab (ball bounced off / rolled away): keep the intake
//            // on, but give up waiting past the confirm window instead of
//            // crediting a pickup the robot never made. A window <= 0 is a
//            // ONE-SHOT check at the end of the dwell - never wait forever.
//            if (HiveConfig.PICKUP_CONFIRM_MS <= 0
//                    || stateTimer.milliseconds() >= HiveConfig.PICKUP_DWELL_MS + HiveConfig.PICKUP_CONFIRM_MS) {
//                failedPickups++;
//                tracker.reset();
//                enter(State.SCAN);   // drop the lock and re-plan / retry
//            }
//            return;
//        }
//
//        pickups++;
//        failedApproaches = 0;        // a real pickup proves the field is reachable again
//        double cx, cy;
//        boolean haveCoords = true;
//        if (haveChaseBall) { cx = lastChaseX; cy = lastChaseY; }
//        else if (currentTarget != null) { cx = currentTarget.x; cy = currentTarget.y; }
//        else { cx = 0; cy = 0; haveCoords = false; }
//
//        if (haveCoords) {
//            for (int i = memory.size() - 1; i >= 0; i--) {
//                Ball b = memory.get(i);
//                if (Math.hypot(b.x - cx, b.y - cy) < HiveConfig.CLEAR_RADIUS) memory.remove(i);
//            }
//        }
//        if (currentTarget != null) memory.remove(currentTarget);
//        currentTarget = null;
//        haveChaseBall = false;
//        tracker.reset();
//
//        if (maxPickups > 0 && pickups >= maxPickups) { finish(); return; }
//        enter(State.SCAN);
//    }
//
//    private void finish() {
//        follower.breakFollowing();
//        follower.startTeleopDrive();
//        teleop = true;
//        drive(0, 0, 0);
//        setIntake(0);
//        state = State.DONE;
//    }
//
//    /* ---------------- geometry ---------------- */
//
//    /** (tx, ty) in degrees -> field (x, y) of a ball on the floor, or null if above the horizon. */
//    private double[] project(double txDeg, double tyDeg, Pose pose) {
//        double p = Math.toRadians(HiveConfig.CAM_PITCH_DEG);
//        double h = HiveConfig.CAM_H - HiveConfig.BALL_H;
//        double tanTx = Math.tan(Math.toRadians(txDeg));
//        double tanTy = Math.tan(Math.toRadians(tyDeg));
//
//        double denom = Math.sin(p) - tanTy * Math.cos(p);
//        if (denom <= 0.02) return null;
//        double s = h / denom;
//
//        double xr = s * (Math.cos(p) + tanTy * Math.sin(p)) + HiveConfig.CAM_X_OFFSET;   // robot frame, forward
//        double yr = -s * tanTx + HiveConfig.CAM_Y_OFFSET;                                // robot frame, left
//
//        double th = pose.getHeading();
//        double fx = pose.getX() + xr * Math.cos(th) - yr * Math.sin(th);
//        double fy = pose.getY() + xr * Math.sin(th) + yr * Math.cos(th);
//        return new double[]{fx, fy};
//    }
//
//    /** Is a ball at this field position inside the usable part of the camera image? */
//    private boolean inView(Ball b, Pose pose) {
//        double dxw = b.x - pose.getX();
//        double dyw = b.y - pose.getY();
//        double th = pose.getHeading();
//        double xr =  dxw * Math.cos(th) + dyw * Math.sin(th);
//        double yr = -dxw * Math.sin(th) + dyw * Math.cos(th);
//
//        double dx = xr - HiveConfig.CAM_X_OFFSET;
//        double dy = yr - HiveConfig.CAM_Y_OFFSET;
//        if (Math.hypot(dx, dy) > HiveConfig.MAX_PLAN_RANGE) return false;
//
//        double p = Math.toRadians(HiveConfig.CAM_PITCH_DEG);
//        double h = HiveConfig.CAM_H - HiveConfig.BALL_H;
//        double zc = dx * Math.cos(p) + h * Math.sin(p);
//        if (zc <= 0) return false;
//        double xc = -dy;
//        double yc = dx * Math.sin(p) - h * Math.cos(p);
//
//        double txp = Math.toDegrees(Math.atan2(xc, zc));
//        double typ = Math.toDegrees(Math.atan2(yc, zc));
//        return Math.abs(txp) < HiveConfig.HFOV_DEG / 2 - HiveConfig.FOV_MARGIN_DEG
//                && Math.abs(typ) < HiveConfig.VFOV_DEG / 2 - HiveConfig.FOV_MARGIN_DEG;
//    }
//
//    private Ball nearest(List<Ball> list, Pose pose) {
//        Ball best = null;
//        double bestD = Double.MAX_VALUE;
//        for (Ball b : list) {
//            double d = Math.hypot(b.x - pose.getX(), b.y - pose.getY());
//            if (d < bestD) { bestD = d; best = b; }
//        }
//        return best;
//    }
//
//    /* ---------------- helpers ---------------- */
//
//    private void enter(State s) {
//        state = s;
//        stateTimer.reset();
//        if (s == State.SCAN) samples.clear();
//        if (s == State.CHASE) {
//            // drop any tracker lock (possibly stale from the travel/settled
//            // frames) so the camera-only final approach starts clean
//            tracker.reset();
//            lastSeen.reset();
//            lastSeenDist = Double.MAX_VALUE;
//            chaseSawBall = false;
//            haveChaseBall = false;
//        }
//        if (s != State.TRAVEL && !teleop) {
//            follower.startTeleopDrive();
//            teleop = true;
//        }
//    }
//
//    /** Robot-centric drive through Pedro (keeps localization running). fwd +, strafe + = LEFT, turn + = CCW. */
//    private void drive(double fwd, double strafe, double turn) {
//        follower.setTeleOpDrive(fwd, strafe, turn, true);
//    }
//
//    private void setIntake(double power) {
//        if (intake != null) intake.setPower(power);
//    }
//
//    private static double wrap(double a) {
//        return Math.atan2(Math.sin(a), Math.cos(a));
//    }
//}