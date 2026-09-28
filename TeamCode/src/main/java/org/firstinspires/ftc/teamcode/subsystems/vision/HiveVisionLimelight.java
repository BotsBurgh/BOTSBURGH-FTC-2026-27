package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.Locale;

public class HiveVisionLimelight {

    private final Limelight3A limelight;

    // Must be the Neural Detector pipeline with the 3A .tflite + labels.txt uploaded
    private static final int PIPELINE = 0;

    // Ignore detections below this confidence (0-1)
    private static final double MIN_CONFIDENCE = 0.40;

    private LLResult latestResult;

    public HiveVisionLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    public void update() {
        latestResult = limelight.getLatestResult();
    }

    public void stop() {
        limelight.stop();
    }

    public Detection getBestDetection() {
        return getDetection(null);
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
     * Highest-confidence detection whose class name STARTS WITH the prefix
     * (matches "yellow", "yellow_pollen", "Yellow", etc.).
     * Pass null to accept any class.
     */
    private Detection getDetection(String prefix) {

        if (!isValidFrame()) {
            return new Detection();
        }

        List<LLResultTypes.DetectorResult> detections =
                latestResult.getDetectorResults();

        if (detections == null || detections.isEmpty()) {
            return new Detection();
        }

        LLResultTypes.DetectorResult best = null;

        for (LLResultTypes.DetectorResult d : detections) {

            if (d.getConfidence() < MIN_CONFIDENCE) {
                continue;
            }

            if (prefix != null) {
                String name = d.getClassName();
                if (name == null ||
                        !name.toLowerCase(Locale.US).startsWith(prefix)) {
                    continue;
                }
            }

            if (best == null || d.getConfidence() > best.getConfidence()) {
                best = d;
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

    /** Raw view of what the Limelight is returning, ignoring filters. */
    public String debugClasses() {

        if (!isValidFrame()) {
            return "invalid frame (check pipeline / connection)";
        }

        List<LLResultTypes.DetectorResult> d = latestResult.getDetectorResults();

        if (d == null || d.isEmpty()) {
            return "valid frame, 0 detections (check model / pipeline type)";
        }

        StringBuilder sb = new StringBuilder();
        for (LLResultTypes.DetectorResult r : d) {
            sb.append(r.getClassName())
                    .append(" ")
                    .append(String.format(Locale.US, "%.2f", r.getConfidence()))
                    .append(", ");
        }
        return sb.toString();
    }

    private boolean isValidFrame() {
        return latestResult != null && latestResult.isValid();
    }

    public static class Detection {

        public String className = "";
        public double confidence = 0;
        public double tx = 0;   // horizontal angle from crosshair
        public double ty = 0;   // vertical angle from crosshair
        public boolean visible = false;

        public Detection() {}

        public Detection(String className, double confidence,
                         double tx, double ty, boolean visible) {
            this.className = className;
            this.confidence = confidence;
            this.tx = tx;
            this.ty = ty;
            this.visible = visible;
        }
    }
}