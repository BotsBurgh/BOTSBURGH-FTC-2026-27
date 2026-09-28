package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * (Potentiometer turned fully clockwise, LED glows blue.)
 * Detects balls and counts them. Call update() every loop.
 */
public class BallSensor {

    // Name of the "Analog Input" device in the Robot Configuration
    private static final String SENSOR_NAME = "ballSensor";

    private static final double MAX_RANGE_MM = 100;

    // Ball counts as present when closer than this (mm). TUNE with telemetry.
    private static final double BALL_THRESHOLD_MM =15;

    // Must read "present" or "absent" this many loops in a row to change state.
    // Raise if you get double counts.
    private static final int DEBOUNCE_LOOPS = 3;

    private static final int MAX_BALLS = 4;

    private final AnalogInput sensor;

    private double distanceMm = MAX_RANGE_MM;
    private boolean ballPresent = false;
    private int streak = 0;
    private int ballCount = 0;

    public BallSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(AnalogInput.class, SENSOR_NAME);
    }

    public void update() {

        distanceMm = sensor.getVoltage() / sensor.getMaxVoltage() * MAX_RANGE_MM;

        boolean rawPresent = distanceMm < BALL_THRESHOLD_MM;

        if (rawPresent == ballPresent) {
            streak = 0;
            return;
        }

        streak++;

        if (streak >= DEBOUNCE_LOOPS) {
            ballPresent = rawPresent;
            streak = 0;

            // Count the moment a ball arrives (rising edge)
            if (ballPresent && ballCount < MAX_BALLS) {
                ballCount++;
            }
        }
    }

    public boolean hasBall() {
        return ballPresent;
    }

    public int getBallCount() {
        return ballCount;
    }

    public boolean isFull() {
        return ballCount >= MAX_BALLS;
    }

    public double getDistanceMm() {
        return distanceMm;
    }

    public double getVoltage() {
        return sensor.getVoltage();
    }

    /** Call when a ball is shot out. */
    public void ballShot() {
        if (ballCount > 0) {
            ballCount--;
        }
    }

    public void resetCount() {
        ballCount = 0;
    }
}