package org.firstinspires.ftc.teamcode.vision2.blobvision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Runs AutoSteps one after another, without blocking. Bolt it onto any existing autonomous:
 *
 * <pre>
 *   chain = StepChain.create()
 *       .thenRun(() -> intake.on())
 *       .then(new BlobCollectStep(camera, drive, cfg).color(BlobColor.PURPLE).collect(2).forSeconds(6))
 *       .thenRun(() -> intake.off());
 *
 *   // in loop(), in whatever state of your auto should run it:
 *   if (chain.update()) { goToNextAutoState(); }
 * </pre>
 *
 * A chain is itself an AutoStep, so chains nest.
 */
public final class StepChain implements AutoStep {

    private final List<AutoStep> steps = new ArrayList<>();
    private int idx = 0;
    private boolean stepStarted = false;
    private boolean started = false;

    public static StepChain create() {
        return new StepChain();
    }

    public StepChain then(AutoStep step) {
        steps.add(step);
        return this;
    }

    /** Runs the Runnable once, instantly. */
    public StepChain thenRun(final Runnable r) {
        return then(new AutoStep() {
            @Override public void start() { r.run(); }
            @Override public boolean update() { return true; }
            @Override public void stop() {}
        });
    }

    /** Waits (non-blocking) for the given time. */
    public StepChain thenWaitMs(final long ms) {
        return then(new AutoStep() {
            private long t0;
            @Override public void start() { t0 = System.nanoTime() / 1_000_000L; }
            @Override public boolean update() { return System.nanoTime() / 1_000_000L - t0 >= ms; }
            @Override public void stop() {}
        });
    }

    /** Waits until the condition is true. */
    public StepChain thenWaitUntil(final BooleanSupplier cond) {
        return then(new AutoStep() {
            @Override public void start() {}
            @Override public boolean update() { return cond.getAsBoolean(); }
            @Override public void stop() {}
        });
    }

    // ------------------------------------------------------------------ AutoStep

    @Override
    public void start() {
        idx = 0;
        stepStarted = false;
        started = true;
    }

    /** @return true once every step has finished. Steps that finish instantly are chained in one call. */
    @Override
    public boolean update() {
        if (!started) start();
        while (idx < steps.size()) {
            AutoStep s = steps.get(idx);
            if (!stepStarted) {
                s.start();
                stepStarted = true;
            }
            if (!s.update()) return false;
            s.stop();
            idx++;
            stepStarted = false;
        }
        return true;
    }

    /** Stops the running step and abandons the rest. */
    @Override
    public void stop() {
        if (stepStarted && idx < steps.size()) steps.get(idx).stop();
        idx = steps.size();
        stepStarted = false;
    }

    public boolean isFinished() { return started && idx >= steps.size(); }
    public int currentIndex() { return idx; }
    public int size() { return steps.size(); }
}
