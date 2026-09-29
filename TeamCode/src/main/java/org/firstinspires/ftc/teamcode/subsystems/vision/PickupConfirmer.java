/*
 * PickupConfirmer - ONE shared intake-confirmation hook for every chase
 * driver (BallChaseController, BallChaseFollower, BallWrangler). Return true
 * from a color sensor, beam break, or current spike when a ball is actually
 * in the intake. If not set, every dwell is counted as a successful pickup.
 */
package org.firstinspires.ftc.teamcode.subsystems.vision;

@FunctionalInterface
public interface PickupConfirmer {
    boolean isBallInIntake();
}