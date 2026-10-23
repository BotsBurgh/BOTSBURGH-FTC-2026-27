package org.firstinspires.ftc.teamcode.vision2.blobvision;

/**
 * One non-blocking unit of autonomous work. Call {@link #update()} every loop until it returns true.
 */
public interface AutoStep {

    /** Called once, right before the first update(). */
    void start();

    /** @return true when finished. */
    boolean update();

    /** Called once when the step finishes, or when the chain aborts it. Must be safe to call twice. */
    void stop();
}
