package org.firstinspires.ftc.teamcode.vision2.blobvision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Turns noisy per-frame sightings into one stable target:
 * smoothing (EMA), a lock that sticks to the same artifact, multi-frame confirmation,
 * and a "lock dropped" event so callers can tell when an artifact vanished into the intake.
 *
 * Hardware-free: feed it the list from {@link LimelightBlobCamera#update}.
 */
public final class BlobTracker {

    /** Chooses which candidate to lock when nothing is locked. */
    public interface Selector {
        Blob pick(List<Blob> candidates);
    }

    public static final Selector CLOSEST = new Selector() {
        @Override
        public Blob pick(List<Blob> c) {
            Blob best = null;
            double bestScore = Double.MAX_VALUE;
            for (Blob b : c) {
                double s = b.hasRange() ? b.distanceIn : 1000 + Math.abs(b.bearingDeg);
                if (s < bestScore) { bestScore = s; best = b; }
            }
            return best;
        }
    };

    public static final Selector LARGEST = new Selector() {
        @Override
        public Blob pick(List<Blob> c) {
            Blob best = null;
            for (Blob b : c) if (best == null || b.areaPct > best.areaPct) best = b;
            return best;
        }
    };

    public static final Selector CENTERED = new Selector() {
        @Override
        public Blob pick(List<Blob> c) {
            Blob best = null;
            for (Blob b : c) if (best == null || Math.abs(b.bearingDeg) < Math.abs(best.bearingDeg)) best = b;
            return best;
        }
    };

    private final BlobConfig cfg;
    private Selector selector;

    private Blob lock;
    private int frames;
    private long lastSeenMs;
    private boolean seenNow;
    private List<Blob> visible = Collections.emptyList();

    private Blob dropped;            // set when a lock times out; cleared by pollDropped()
    private boolean droppedConfirmed;

    public BlobTracker(BlobConfig cfg) {
        this.cfg = cfg;
        switch (cfg.selectMode) {
            case LARGEST: selector = LARGEST; break;
            case CENTERED: selector = CENTERED; break;
            default: selector = CLOSEST;
        }
    }

    public void setSelector(Selector s) {
        this.selector = s;
    }

    public void update(List<Blob> blobs, long nowMs) {
        visible = new ArrayList<>(blobs);
        seenNow = false;

        if (lock != null) {
            Blob match = nearestTo(lock);
            if (match != null) {
                lock = lock.blend(match, cfg.emaAlpha);
                frames++;
                lastSeenMs = nowMs;
                seenNow = true;
            } else if (nowMs - lastSeenMs > cfg.lostTimeoutMs) {
                dropped = lock;
                droppedConfirmed = frames >= cfg.confirmFrames;
                lock = null;
                frames = 0;
            }
        }

        if (lock == null && !visible.isEmpty()) {
            Blob pick = selector.pick(visible);
            if (pick != null) {
                lock = pick;
                frames = 1;
                lastSeenMs = nowMs;
                seenNow = true;
            }
        }
    }

    private Blob nearestTo(Blob ref) {
        Blob best = null;
        double bestSep = cfg.lockRadiusIn;
        for (Blob b : visible) {
            if (b.color != ref.color) continue;
            double sep = ref.separationTo(b);
            if (sep <= bestSep) { bestSep = sep; best = b; }
        }
        return best;
    }

    // ------------------------------------------------------------------ queries

    /** Smoothed locked target (keeps its last value for lostTimeoutMs after it disappears), or null. */
    public Blob target() { return lock; }
    public boolean hasTarget() { return lock != null; }
    public boolean isConfirmed() { return lock != null && frames >= cfg.confirmFrames; }
    public boolean seenThisFrame() { return seenNow; }
    public List<Blob> visible() { return visible; }

    /** Returns the lock that most recently timed out (once), or null. */
    public Blob pollDropped() {
        Blob d = dropped;
        dropped = null;
        return d;
    }

    /** Whether the lock returned by the last {@link #pollDropped()} had been confirmed. */
    public boolean lastDropWasConfirmed() { return droppedConfirmed; }

    public void clearLock() {
        lock = null;
        frames = 0;
    }

    public void reset() {
        lock = null;
        frames = 0;
        dropped = null;
        droppedConfirmed = false;
        visible = Collections.emptyList();
        seenNow = false;
    }
}
