package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class HiveVisionLimelight {

    private final Limelight3A limelight;

    private static final int PIPELINE = 0;

    public HiveVisionLimelight(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(PIPELINE);

        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public Detection getBestDetection() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return new Detection();
        }

        List<LLResultTypes.DetectorResult> detections =
                result.getDetectorResults();

        if (detections == null || detections.isEmpty()) {
            return new Detection();
        }

        LLResultTypes.DetectorResult best = detections.get(0);

        for (LLResultTypes.DetectorResult detection : detections) {

            if (detection.getConfidence() > best.getConfidence()) {
                best = detection;
            }
        }

        return new Detection(
                best.getClassName(),
                best.getConfidence(),
                best.getTargetXDegrees(),
                best.getTargetYDegrees(),
                true
        );
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

    private Detection getDetection(String wantedClass) {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return new Detection();
        }

        List<LLResultTypes.DetectorResult> detections =
                result.getDetectorResults();

        if (detections == null) {
            return new Detection();
        }

        Detection best = new Detection();

        for (LLResultTypes.DetectorResult detection : detections) {

            String name = detection.getClassName();

            if (name == null) {
                continue;
            }

            if (name.toLowerCase().contains(wantedClass.toLowerCase())) {

                if (!best.visible ||
                        detection.getConfidence() > best.confidence) {

                    best = new Detection(
                            name,
                            detection.getConfidence(),
                            detection.getTargetXDegrees(),
                            detection.getTargetYDegrees(),
                            true
                    );
                }
            }
        }

        return best;
    }

    public static class Detection {

        public String className;
        public double confidence;

        // Degrees from Limelight crosshair
        public double tx;
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
                boolean visible
        ) {
            this.className = className;
            this.confidence = confidence;
            this.tx = tx;
            this.ty = ty;
            this.visible = visible;
        }
    }
}