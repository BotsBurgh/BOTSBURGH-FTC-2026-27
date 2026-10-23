package org.firstinspires.ftc.teamcode.vision2.blobvision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Thin wrapper over the Limelight 3A that returns {@link Blob}s instead of raw results.
 *
 * Color mode : one Limelight color pipeline per color. Target = ANY flips between the pipelines
 *              (see BlobConfig.anyCycleMs) and merges the latest sighting of each.
 * Neural mode: set BlobConfig.detectorPipeline; class names are mapped to colors.
 *
 * Call {@link #start()} once, then {@link #update(long, RobotPose)} every loop.
 */
public final class LimelightBlobCamera {

    private final Limelight3A limelight;
    private final BlobConfig cfg;

    private BlobColor target = BlobColor.ANY;
    private boolean started = false;

    private int activePipeline = -1;
    private long lastSwitchMs = 0;
    private int cycleIdx = 0;

    private final Map<BlobColor, List<Blob>> cache = new HashMap<>();
    private final Map<BlobColor, Long> cacheTime = new HashMap<>();

    private double stalenessMs = Double.NaN;
    private double captureLatencyMs = Double.NaN;
    private double targetingLatencyMs = Double.NaN;
    private int visibleCount = 0;

    public LimelightBlobCamera(HardwareMap hardwareMap, BlobConfig cfg) {
        this(hardwareMap.get(Limelight3A.class, cfg.cameraName), cfg);
    }

    public LimelightBlobCamera(Limelight3A limelight, BlobConfig cfg) {
        this.limelight = limelight;
        this.cfg = cfg;
    }

    /** Idempotent. Starts polling. */
    public void start() {
        if (started) return;
        limelight.setPollRateHz(cfg.pollRateHz);
        limelight.start();
        started = true;
    }

    public void stop() {
        if (!started) return;
        limelight.stop();
        started = false;
        activePipeline = -1;
    }

    public void setTarget(BlobColor color) {
        this.target = color;
        cycleIdx = 0;
    }

    public BlobColor getTarget() {
        return target;
    }

    /** Reads the latest Limelight result and returns every blob currently believed visible. */
    public List<Blob> update(long nowMs, RobotPose pose) {
        LLResult r = limelight.getLatestResult();
        if (r != null) {
            stalenessMs = r.getStaleness();
            captureLatencyMs = r.getCaptureLatency();
            targetingLatencyMs = r.getTargetingLatency();
        }

        List<BlobColor> wanted = wantedColors();
        if (cfg.detectorPipeline >= 0) {
            ensurePipeline(cfg.detectorPipeline, nowMs);
            if (fresh(r) && r.getPipelineIndex() == activePipeline) readDetector(r, nowMs, pose);
        } else {
            readColor(r, wanted, nowMs, pose);
        }

        List<Blob> out = new ArrayList<>();
        long hold = (cfg.detectorPipeline < 0 && wanted.size() > 1)
                ? cfg.anyHoldMs : Math.max(cfg.maxStalenessMs, 100);
        for (BlobColor c : wanted) {
            Long t = cacheTime.get(c);
            List<Blob> l = cache.get(c);
            if (t != null && l != null && nowMs - t <= hold) out.addAll(l);
        }
        visibleCount = out.size();
        return out;
    }

    // ------------------------------------------------------------------ color mode

    private void readColor(LLResult r, List<BlobColor> wanted, long now, RobotPose pose) {
        if (wanted.size() > 1 && now - lastSwitchMs >= cfg.anyCycleMs) {
            cycleIdx = (cycleIdx + 1) % wanted.size();
        }
        BlobColor want = wanted.get(cycleIdx % wanted.size());
        ensurePipeline(want.pipeline, now);

        if (!fresh(r) || r.getPipelineIndex() != activePipeline) return;

        List<Blob> found = new ArrayList<>();
        for (LLResultTypes.ColorResult c : r.getColorResults()) {
            double area = c.getTargetArea();
            if (area < cfg.minAreaPct || area > cfg.maxAreaPct) continue;
            Blob b = BlobMath.makeBlob(cfg, want, c.getTargetXDegrees(), c.getTargetYDegrees(), area, 1.0, now);
            if (b.hasRange() && b.cameraDistanceIn > cfg.maxRangeIn) continue;
            found.add(project(b, pose));
        }
        cache.put(want, found);
        cacheTime.put(want, now);
    }

    // ------------------------------------------------------------------ neural mode

    private void readDetector(LLResult r, long now, RobotPose pose) {
        Map<BlobColor, List<Blob>> byColor = new HashMap<>();
        for (BlobColor c : cfg.colors) byColor.put(c, new ArrayList<Blob>());

        for (LLResultTypes.DetectorResult d : r.getDetectorResults()) {
            BlobColor col = classify(d.getClassName());
            if (col == null) continue;
            if (!target.isAny() && col != target) continue;

            double conf = d.getConfidence();
            if (conf > 1.0) conf /= 100.0; // some firmware reports percent
            if (conf < cfg.minConfidence) continue;

            double area = d.getTargetArea();
            if (area < cfg.minAreaPct || area > cfg.maxAreaPct) continue;

            Blob b = BlobMath.makeBlob(cfg, col, d.getTargetXDegrees(), d.getTargetYDegrees(), area, conf, now);
            if (b.hasRange() && b.cameraDistanceIn > cfg.maxRangeIn) continue;
            byColor.get(col).add(project(b, pose));
        }
        for (Map.Entry<BlobColor, List<Blob>> e : byColor.entrySet()) {
            cache.put(e.getKey(), e.getValue());
            cacheTime.put(e.getKey(), now);
        }
    }

    private BlobColor classify(String className) {
        for (BlobColor c : cfg.colors) if (c.matchesClass(className)) return c;
        return null;
    }

    // ------------------------------------------------------------------ helpers

    private List<BlobColor> wantedColors() {
        List<BlobColor> w = new ArrayList<>();
        if (target.isAny()) w.addAll(cfg.colors);
        else w.add(target);
        if (w.isEmpty()) w.add(BlobColor.PURPLE);
        return w;
    }

    private void ensurePipeline(int pipeline, long now) {
        if (pipeline < 0 || pipeline == activePipeline) return;
        limelight.pipelineSwitch(pipeline);
        activePipeline = pipeline;
        lastSwitchMs = now;
    }

    private boolean fresh(LLResult r) {
        return r != null && r.getStaleness() <= cfg.maxStalenessMs;
    }

    private Blob project(Blob b, RobotPose pose) {
        if (pose == null || !b.hasRange()) return b;
        double[] f = BlobMath.toField(pose, b.forwardIn, b.leftIn);
        return b.withField(f[0], f[1]);
    }

    // ------------------------------------------------------------------ diagnostics

    public int getActivePipeline() { return activePipeline; }
    public double getStalenessMs() { return stalenessMs; }
    public double getCaptureLatencyMs() { return captureLatencyMs; }
    public double getTargetingLatencyMs() { return targetingLatencyMs; }
    public int getVisibleCount() { return visibleCount; }
    public Limelight3A getLimelight() { return limelight; }
}
