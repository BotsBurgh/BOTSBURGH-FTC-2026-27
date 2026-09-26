package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class HiveVisionLimelight {

    private final Limelight3A limelight;

    private static final int PIPELINE = 0;

    // The most recent Limelight frame
    private LLResult latestResult;

    public HiveVisionLimelight(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    public void update() {
        latestResult = limelight.getLatestResult();
    }

    public void stop() {
        limelight.stop();
    }

    /**
     * Gets the highest-confidence detection of ANY class.
     */
    public Detection getBestDetection() {

        if (!isValidFrame()) {
            return new Detection();
        }

        List<LLResultTypes.DetectorResult> detections =
                latestResult.getDetectorResults();

        if (detections == null || detections.isEmpty()) {
            return new Detection();
        }

        LLResultTypes.DetectorResult best = detections.get(0);

        for (LLResultTypes.DetectorResult detection : detections) {

            if (detection.getConfidence() > best.getConfidence()) {
                best = detection;
            }
        }

        return makeDetection(best);
    }

    public Detection getYellowPollen() {
        return getDetection("yellow");
    }

    public Detection getRedNectar() {
        return getDetection("red");
    }

    public Detection getBlueNectar() {
        return getDetection("blue");
    }

    /**
     * Finds the highest-confidence detection matching the requested class.
     */
    private Detection getDetection(String wantedClass) {

        if (!isValidFrame()) {
            return new Detection();
        }

        List<LLResultTypes.DetectorResult> detections =
                latestResult.getDetectorResults();

        if (detections == null || detections.isEmpty()) {
            return new Detection();
        }

        LLResultTypes.DetectorResult best = null;

        for (LLResultTypes.DetectorResult detection : detections) {

            String name = detection.getClassName();

            if (name == null) {
                continue;
            }

            if (!name.equalsIgnoreCase(wantedClass)) {
                continue;
            }

            if (best == null ||
                    detection.getConfidence() > best.getConfidence()) {

                best = detection;
            }
        }

        if (best == null) {
            return new Detection();
        }

        return new Detection(
                best.getClassName(),
                best.getConfidence(),
                best.getTargetXDegrees(),
                best.getTargetYDegrees(),
                true
        );
    }

    private Detection makeDetection(
            LLResultTypes.DetectorResult detection) {

        return new Detection(
                detection.getClassName(),
                detection.getConfidence(),
                detection.getTargetXDegrees(),
                detection.getTargetYDegrees(),
                true
        );
    }

    private boolean isValidFrame() {

        return latestResult != null &&
                latestResult.isValid();
    }

    public static class Detection {

        public String className;
        public double confidence;

        // Horizontal angle from Limelight crosshair
        public double tx;

        // Vertical angle from Limelight crosshair
        public double ty;

        public boolean visible;

        public Detection() {

            className = "";
            confidence = 0;

            tx = 0;
            ty = 0;

            visible = false;
        }

        public Detection(
                String className,
                double confidence,
                double tx,
                double ty,
                boolean visible) {

            this.className = className;
            this.confidence = confidence;

            this.tx = tx;
            this.ty = ty;

            this.visible = visible;
        }
    }
}