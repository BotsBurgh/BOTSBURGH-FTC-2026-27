package org.firstinspires.ftc.teamcode.vision2.blobvision;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * Telemetry printers. Call whichever you want from any OpMode - they only add data,
 * you call telemetry.update() yourself.
 */
public final class BlobTelemetry {

    private BlobTelemetry() {}

    private static String f(double v, String fmt) {
        return Double.isNaN(v) ? "--" : String.format(fmt, v);
    }

    /** Camera health: pipeline, staleness, latency, how many blobs are visible. */
    public static void addCamera(Telemetry t, LimelightBlobCamera cam) {
        t.addData("LL pipeline", cam.getActivePipeline());
        t.addData("LL age / latency (ms)", "%s / %s+%s",
                f(cam.getStalenessMs(), "%.0f"),
                f(cam.getCaptureLatencyMs(), "%.1f"),
                f(cam.getTargetingLatencyMs(), "%.1f"));
        t.addData("Blobs visible", cam.getVisibleCount());
    }

    /** The locked target in full detail: distance, bearing, raw tx/ty/area, robot + field position. */
    public static void addTarget(Telemetry t, BlobTracker tracker) {
        Blob b = tracker.target();
        if (b == null) {
            t.addData("Target", "none");
            return;
        }
        t.addData("Target", "%s %s%s", b.color,
                tracker.isConfirmed() ? "CONFIRMED" : "acquiring",
                tracker.seenThisFrame() ? "" : " (coasting)");
        t.addData("  Distance (robot ctr)", "%s in", f(b.distanceIn, "%.1f"));
        t.addData("  Distance (lens)", "%s in", f(b.cameraDistanceIn, "%.1f"));
        t.addData("  Bearing", "%s deg (+left)", f(b.bearingDeg, "%.1f"));
        t.addData("  tx / ty / area", "%.1f / %.1f / %.2f%%", b.txDeg, b.tyDeg, b.areaPct);
        t.addData("  Robot frame fwd/left", "%s / %s in", f(b.forwardIn, "%.1f"), f(b.leftIn, "%.1f"));
        t.addData("  Field x/y", "%s / %s in", f(b.fieldX, "%.1f"), f(b.fieldY, "%.1f"));
    }

    /** One line per visible blob (up to max). */
    public static void addAll(Telemetry t, BlobTracker tracker, int max) {
        List<Blob> v = tracker.visible();
        for (int i = 0; i < v.size() && i < max; i++) {
            t.addData("Blob " + i, v.get(i).toString());
        }
    }

    /** Status of a running collect step: state, count, time left. */
    public static void addCollect(Telemetry t, BlobCollectStep s) {
        String goal = s.getTargetCount() == Integer.MAX_VALUE ? "inf" : String.valueOf(s.getTargetCount());
        String left = s.getTimeLimitMs() == Long.MAX_VALUE ? "inf"
                : String.format("%.1f s", s.getRemainingMs() / 1000.0);
        t.addData("Collect state", s.getState());
        t.addData("Collected", "%d / %s (%s)", s.getCollected(), goal, s.getColor());
        t.addData("Time left", left);
        if (s.getFinishReason() != BlobCollectStep.FinishReason.NONE) {
            t.addData("Finished because", s.getFinishReason());
        }
    }
}
