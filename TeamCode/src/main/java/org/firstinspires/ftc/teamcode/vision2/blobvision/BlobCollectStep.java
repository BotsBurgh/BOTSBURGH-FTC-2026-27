package org.firstinspires.ftc.teamcode.vision2.blobvision;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * The main piece: search for artifacts, lock one, drive at it, push it into the intake, repeat -
 * until either {@code collect(n)} artifacts were picked up or {@code forSeconds(t)} ran out.
 * Whichever comes first ends the step. Non-blocking: call update() every loop.
 *
 * <pre>
 *   new BlobCollectStep(camera, drive, cfg)
 *       .color(BlobColor.PURPLE)          // or BlobColor.ANY
 *       .collect(3)                       // artifact limit
 *       .forSeconds(7)                    // time limit
 *       .intake(() -> intake.on(), () -> intake.off())
 *       .pickup(PickupDetector.sensor(() -> beam.isBroken(), 80))   // optional but recommended
 *       .telemetry(telemetry, false)      // prints distance etc.; you call telemetry.update()
 * </pre>
 *
 * {@code trackOnly()} runs the same tracking/telemetry/timer without ever touching the drivetrain.
 */
public final class BlobCollectStep implements AutoStep {

    public enum State { IDLE, SEARCH, CHASE, PUSH, DONE }
    public enum FinishReason { NONE, COUNT_REACHED, TIMEOUT, ABORTED }

    private final LimelightBlobCamera camera;
    private final BlobDrive drive;
    private final BlobConfig cfg;
    private final BlobTracker tracker;
    private BlobPlanner planner = new BlobPlanner();

    private BlobColor color = BlobColor.ANY;
    private int targetCount = Integer.MAX_VALUE;
    private long timeLimitMs = Long.MAX_VALUE;
    private boolean driveEnabled = true;
    private PickupDetector pickup = PickupDetector.INFERRED;
    private Runnable intakeOn, intakeOff, onStart, onFinish;
    private Telemetry telemetry;
    private boolean telemetryAutoUpdate = false;

    // runtime
    private State state = State.IDLE;
    private FinishReason reason = FinishReason.NONE;
    private long startMs, endMs, stateEnterMs;
    private int collected;
    private int searchDir = 1;
    private RobotPose lastPose;
    private boolean pushHadConfirmedTarget;

    public BlobCollectStep(LimelightBlobCamera camera, BlobDrive drive, BlobConfig cfg) {
        this.camera = camera;
        this.drive = drive;
        this.cfg = cfg;
        this.tracker = new BlobTracker(cfg);
        this.tracker.setSelector(new BlobTracker.Selector() {
            @Override
            public Blob pick(List<Blob> candidates) {
                return choose(candidates);
            }
        });
    }

    // ------------------------------------------------------------------ configuration

    public BlobCollectStep color(BlobColor c) { color = c; return this; }
    public BlobCollectStep collect(int n) { targetCount = n; return this; }
    public BlobCollectStep forSeconds(double s) { timeLimitMs = (long) (s * 1000); return this; }
    public BlobCollectStep forMillis(long ms) { timeLimitMs = ms; return this; }
    public BlobCollectStep intake(Runnable on, Runnable off) { intakeOn = on; intakeOff = off; return this; }
    public BlobCollectStep onStart(Runnable r) { onStart = r; return this; }
    public BlobCollectStep onFinish(Runnable r) { onFinish = r; return this; }
    public BlobCollectStep pickup(PickupDetector p) { pickup = p; return this; }
    public BlobCollectStep planner(BlobPlanner p) { planner = p; return this; }
    /** Add a circular field obstacle (needs a pose from the BlobDrive). */
    public BlobCollectStep obstacle(double x, double y, double radius) { planner.obstacle(x, y, radius); return this; }
    /** Track and print only; never command the drivetrain. */
    public BlobCollectStep trackOnly() { driveEnabled = false; return this; }
    public BlobCollectStep telemetry(Telemetry t, boolean callUpdate) {
        telemetry = t;
        telemetryAutoUpdate = callUpdate;
        return this;
    }

    // ------------------------------------------------------------------ AutoStep

    @Override
    public void start() {
        tracker.reset();
        camera.setTarget(color);
        camera.start();
        collected = 0;
        reason = FinishReason.NONE;
        startMs = now();
        searchDir = 1;
        pushHadConfirmedTarget = false;
        enter(State.SEARCH);
        if (onStart != null) onStart.run();
        if (intakeOn != null) intakeOn.run();
    }

    @Override
    public boolean update() {
        if (state == State.DONE) return true;
        if (state == State.IDLE) start();

        long now = now();
        if (now - startMs >= timeLimitMs) {
            finish(FinishReason.TIMEOUT);
            return true;
        }

        lastPose = drive.pose();
        tracker.update(camera.update(now, lastPose), now);

        boolean pushFinished = false;
        boolean pushConfirmed = false;
        BlobControl.Command cmd = BlobControl.Command.ZERO;

        // Lock timed out? If it was a confirmed artifact that vanished close to us, it's under the robot.
        Blob dropped = tracker.pollDropped();
        if (state == State.CHASE && dropped != null && tracker.lastDropWasConfirmed()
                && dropped.hasRange() && dropped.distanceIn <= cfg.blindDistanceIn) {
            pushHadConfirmedTarget = true;
            enter(State.PUSH);
        }

        switch (state) {
            case SEARCH:
                if (tracker.isConfirmed()) {
                    enter(State.CHASE);
                    cmd = chaseCommand(tracker.target());
                } else {
                    cmd = BlobControl.search(cfg, now - stateEnterMs, searchDir);
                }
                break;

            case CHASE: {
                Blob t = tracker.target();
                if (t == null) {
                    enter(State.SEARCH);
                    cmd = BlobControl.search(cfg, 0, searchDir);
                } else if (t.hasRange() && t.distanceIn <= cfg.engageDistanceIn) {
                    searchDir = t.bearingDeg >= 0 ? 1 : -1;
                    pushHadConfirmedTarget = tracker.isConfirmed();
                    enter(State.PUSH);
                    cmd = BlobControl.push(cfg);
                } else {
                    searchDir = t.bearingDeg >= 0 ? 1 : -1;
                    cmd = chaseCommand(t);
                }
                break;
            }

            case PUSH:
                cmd = BlobControl.push(cfg);
                if (now - stateEnterMs >= cfg.pushMs) {
                    pushFinished = true;
                    pushConfirmed = pushHadConfirmedTarget;
                }
                break;

            default:
                break;
        }

        // Count pickups
        int n = pickup.poll(new PickupDetector.Context(pushFinished, pushConfirmed, now));
        if (n > 0) {
            collected += n;
            if (state == State.PUSH) pushFinished = true; // sensor says it's in: stop pushing early
        }
        if (pushFinished) {
            tracker.clearLock();
            enter(State.SEARCH);
            cmd = BlobControl.Command.ZERO;
        }

        if (collected >= targetCount) {
            finish(FinishReason.COUNT_REACHED);
            return true;
        }

        if (driveEnabled) drive.drive(cmd);
        printTelemetry();
        return false;
    }

    @Override
    public void stop() {
        if (state != State.DONE && state != State.IDLE) finish(FinishReason.ABORTED);
    }

    /** Ends the step early from outside (e.g. a global auto timer). */
    public void abort() {
        stop();
    }

    // ------------------------------------------------------------------ internals

    private BlobControl.Command chaseCommand(Blob t) {
        return t == null ? BlobControl.Command.ZERO : BlobControl.chase(cfg, t);
    }

    /** Planner-backed target choice: nearest cheapest-to-reach, obstacle-aware. */
    private Blob choose(List<Blob> candidates) {
        BlobPlanner.Result r = planner.plan(lastPose, candidates);
        if (!r.order.isEmpty()) return r.order.get(0);

        // Fall back to blobs the planner couldn't place (no range): aim at the most centered one.
        Blob best = null;
        List<Blob> unranged = new ArrayList<>();
        for (Blob b : candidates) if (!b.hasRange()) unranged.add(b);
        for (Blob b : unranged) {
            if (best == null || Math.abs(b.bearingDeg) < Math.abs(best.bearingDeg)) best = b;
        }
        return best;
    }

    private void enter(State s) {
        state = s;
        stateEnterMs = now();
    }

    private void finish(FinishReason why) {
        if (state == State.DONE) return;
        reason = why;
        endMs = now();
        state = State.DONE;
        if (driveEnabled) drive.stop();
        if (intakeOff != null) intakeOff.run();
        if (onFinish != null) onFinish.run();
        printTelemetry();
    }

    private void printTelemetry() {
        if (telemetry == null) return;
        BlobTelemetry.addCollect(telemetry, this);
        BlobTelemetry.addTarget(telemetry, tracker);
        BlobTelemetry.addCamera(telemetry, camera);
        if (telemetryAutoUpdate) telemetry.update();
    }

    private static long now() {
        return System.nanoTime() / 1_000_000L;
    }

    // ------------------------------------------------------------------ getters

    public State getState() { return state; }
    public FinishReason getFinishReason() { return reason; }
    public int getCollected() { return collected; }
    public int getTargetCount() { return targetCount; }
    public long getTimeLimitMs() { return timeLimitMs; }
    public BlobColor getColor() { return color; }
    public BlobTracker getTracker() { return tracker; }
    public boolean isFinished() { return state == State.DONE; }

    public long getElapsedMs() {
        if (state == State.IDLE) return 0;
        return (state == State.DONE ? endMs : now()) - startMs;
    }

    public long getRemainingMs() {
        if (timeLimitMs == Long.MAX_VALUE) return Long.MAX_VALUE;
        return Math.max(0, timeLimitMs - getElapsedMs());
    }
}
