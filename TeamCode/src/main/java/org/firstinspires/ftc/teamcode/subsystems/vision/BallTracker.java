/*
 * BallTracker - perception core shared by every ball-chase driver: raw
 * Limelight 3A detections become ONE chosen ball per frame via a color-bound
 * target lock. Class docs moved to MODULES.md (#balltracker).
 */
package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.vision.HiveConfig;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class BallTracker {

    /* All tunables now live in ONE file: HiveConfig. This class reads them at
     * point of use, so editing HiveConfig (or a config system driving it at
     * runtime) takes effect immediately - nothing is copied here at load. */

    /* ---- class ids - CONFIRM against the pipeline label list, not this comment ---- */
    public static final int CLASS_YELLOW_NEUTRAL = HiveConfig.CLASS_YELLOW_NEUTRAL;
    public static final int CLASS_RED            = HiveConfig.CLASS_RED;
    public static final int CLASS_BLUE           = HiveConfig.CLASS_BLUE;
    public static final Set<Integer> CLASSES_RED_BLUE = setOf(CLASS_RED, CLASS_BLUE);
    public static final Set<Integer> CLASSES_ALL      = setOf(CLASS_RED, CLASS_BLUE, CLASS_YELLOW_NEUTRAL);

    /** One chosen ball this frame. `predicted` sightings carry last-known angles
     *  while the locked ball is momentarily missing (fresh lock only). */
    public static class Sighting {
        public final int    classId;
        public final double txDeg;        // + to the RIGHT
        public final double tyDeg;        // + UP
        public final double confidence;
        public final double distIn;       // groundRange(tyDeg); 999.0 if near the horizon
        public final boolean predicted;

        public Sighting(int classId, double txDeg, double tyDeg, double confidence, boolean predicted, double distIn) {
            this.classId    = classId;
            this.txDeg      = txDeg;
            this.tyDeg      = tyDeg;
            this.confidence = confidence;
            this.predicted  = predicted;
            this.distIn     = distIn;
        }
    }

    /** Off-SDK sighting datum holding exactly what the tracker consumes, so a
     *  scripted DetectionSource can drive the full lock logic on a laptop
     *  (unit tests) with no Limelight in sight. */
    public static final class RawDet {
        public final int    classId;
        public final double txDeg, tyDeg, confidence, area;
        public RawDet(int classId, double txDeg, double tyDeg, double confidence, double area) {
            this.classId    = classId;
            this.txDeg      = txDeg;
            this.tyDeg      = tyDeg;
            this.confidence = confidence;
            this.area       = area;
        }
    }

    /** Feeds the tracker raw detections + staleness. The default implementation
     *  reads the Limelight; tests (and field-playback tools) script frames.
     *  latest() returns an EMPTY list (never null) when nothing is visible;
     *  stalenessMs() returns Long.MAX_VALUE when there is no data at all. */
    public interface DetectionSource {
        List<RawDet> latest();
        long stalenessMs();
        /** Monotonic frame identifier from the camera, so consumers can tell a
         *  RE-used result from a NEW frame (confirmation counts distinct
         *  frames, not update() calls; the follower's SCAN sampler only takes
         *  genuinely fresh frames). LimelightSource uses the LL result
         *  timestamp; a scripted source bumps it per synthetic frame. */
        long frame();
    }

    private static final class LimelightSource implements DetectionSource {
        private final Limelight3A limelight;
        LimelightSource(Limelight3A limelight) { this.limelight = limelight; }

        @Override public List<RawDet> latest() {
            LLResult r = limelight == null ? null : limelight.getLatestResult();
            List<LLResultTypes.DetectorResult> d = r == null ? null : r.getDetectorResults();
            if (d == null || d.isEmpty()) return java.util.Collections.emptyList();
            List<RawDet> out = new java.util.ArrayList<>(d.size());
            for (LLResultTypes.DetectorResult det : d) {
                out.add(new RawDet(det.getClassId(), det.getTargetXDegrees(),
                        det.getTargetYDegrees(), det.getConfidence(), det.getTargetArea()));
            }
            return out;
        }

        @Override public long stalenessMs() {
            LLResult r = limelight == null ? null : limelight.getLatestResult();
            return r == null ? Long.MAX_VALUE : r.getStaleness();
        }

        @Override public long frame() {
            LLResult r = limelight == null ? null : limelight.getLatestResult();
            return r == null ? 0L : (long) r.getTimestamp();
        }
    }

    private final DetectionSource source;
    private Set<Integer> allowed = new HashSet<>(CLASSES_RED_BLUE);

    private Double lockTx = null, lockTy = null;   // last-known angles of the locked ball
    private Integer lockClass = null;               // class the lock is bound to (null = lock-free)
    private final ElapsedTime lockAge = new ElapsedTime();
    private Sighting locked = null;                 // last real sighting of the locked ball
    private Sighting last = null;                   // most recent result of update()

    /* ---- multi-frame confirmation state ---- */
    private int    confirmFrames  = 0;              // per-instance override; -1 = explicitly OFF, 0 = use HiveConfig
    private double confirmGateDeg = 0;              // per-instance override; 0 = use HiveConfig
    private int    confirmStreak  = 0;              // consecutive frames the candidate ball has been seen
    private long   lastConfirmFrame = -1;           // frame id the current confirm streak was last advanced on
    private Integer confirmClass = null;            // candidate's class (null = no candidate yet)
    private Double  confirmTx = null, confirmTy = null; // candidate's last angular position

    public BallTracker(Limelight3A limelight) {
        this(new LimelightSource(limelight));
    }

    public BallTracker(DetectionSource source) {
        this.source = source;
    }

    /* ---------------- public API ---------------- */

    /** Which classes this tracker will chase. Replaces the whole set and clears any lock. */
    public void setAllowedClasses(Set<Integer> classes) {
        reset();
        this.allowed = new HashSet<>(classes);
    }

    public void setAllowedClasses(Integer... classes) {
        setAllowedClasses(setOf(classes));
    }

    public void addAllowedClass(int classId) {
        allowed.add(classId);
    }

    public boolean isAllowed(int classId) {
        return allowed.contains(classId);
    }

    public Set<Integer> getAllowedClasses() {
        return allowed;
    }

    /** Drop the target lock (call when starting a fresh hunt). */
    public void reset() {
        lockClass = null;
        lockTx = null;
        lockTy = null;
        locked = null;
        last = null;
        resetCandidate();
    }

    /** Arm multi-frame confirmation: a ball must be seen `frames` CONSECUTIVE
     *  DISTINCT camera frames (same class, angularly within the gate of its
     *  last position) before `update()` adopts it as the locked target — a
     *  deliberate guard against single-frame false positives (e.g. red robot
     *  panels). The count advances on a new DetectionSource frame() only, so
     *  re-polling the same camera frame cannot inflate the streak. Callers can
     *  hold off acting via hasConfirmedTarget(). Pass 0 (or less) to turn it
     *  OFF even when HiveConfig.CONFIRM_FRAMES is armed. Returns this tracker
     *  for fluency: `tracker.requireConfirmation(4, 12.0);` */
    public BallTracker requireConfirmation(int frames) {
        this.confirmFrames = frames <= 0 ? -1 : frames;   // 0 = explicit OFF, not "use HiveConfig"
        return this;
    }

    public BallTracker requireConfirmation(int frames, double gateDeg) {
        requireConfirmation(frames);
        if (gateDeg > 0) this.confirmGateDeg = gateDeg;
        return this;
    }

    /** True while a REAL target track is live. Note `locked` only ever holds
     *  real sightings (never `predicted` coasters), so this doubles as the
     *  "same ball survived confirmation" gate: while confirmation is armed and
     *  the tracker is still counting frames, `locked` is null and this is false.
     *  With confirmation off, a first confident detection is confirmed at once,
     *  so `if (tracker.hasConfirmedTarget()) chase();` works either way. */
    public boolean hasConfirmedTarget() {
        return locked != null;
    }

    /** Consecutive frames the current candidate has already passed.
     *  0 when confirmation is off or no candidate is being counted. */
    public int getConfirmStreak() {
        return confirmStreak;
    }

    /** Frames this tracker currently requires before adopting a target
     *  (per-instance override, else the HiveConfig.CONFIRM_FRAMES default;
     *  0 when confirmation is explicitly disarmed). */
    public int getConfirmationFrames() {
        return effConfirmFrames();
    }

    /** Poll the Limelight and pick this frame's ball. Returns null only when
     *  nothing usable is visible AND no fresh lock exists to coast on. */
    public Sighting update() {
        last = compute();
        return last;
    }

    public Sighting getLast() {
        return last;
    }

    /** Frame identity of the most recent camera result (0 if none yet). Lets
     *  callers tell a re-polled/re-used frame from a genuinely new one. */
    public long getFrame() {
        return source.frame();
    }

    /** Staleness of the latest Limelight result in milliseconds (for scan-time gating). */
    public long getStalenessMs() {
        return source.stalenessMs();
    }

    /** Confidence- and class-gated detections for scan-time field projection.
     *  Null if the result is stale or missing. Does NOT touch the target lock. */
    public List<RawDet> getGatedDetections() {
        if (source.stalenessMs() > HiveConfig.MAX_STALENESS_MS) return null;
        List<RawDet> dets = source.latest();
        if (dets == null || dets.isEmpty()) return null;

        List<RawDet> out = new java.util.ArrayList<>();
        for (RawDet d : dets) {
            if (!allowed.contains(d.classId)) continue;
            if (d.confidence < HiveConfig.MIN_CONF) continue;
            out.add(d);
        }
        return out;
    }

    /** Every confident detection, whatever the class (still staleness-gated).
     *  Unlike getGatedDetections() this ignores the allowed-class filter, so an
     *  explicit "find that color" can peek at any class. Null when stale/missing. */
    public List<RawDet> getConfidentResults() {
        if (source.stalenessMs() > HiveConfig.MAX_STALENESS_MS) return null;
        List<RawDet> dets = source.latest();
        if (dets == null || dets.isEmpty()) return null;

        List<RawDet> out = new java.util.ArrayList<>();
        for (RawDet d : dets) {
            if (d.confidence < HiveConfig.MIN_CONF) continue;
            out.add(d);
        }
        return out;
    }

    /** Make update() chase THIS ball (class + last-known angles) exactly, for the
     *  verb wrapper's grab(target) / alignTo(target). Folds straight into the
     *  existing target lock: real detections within LOCK_GATE_DEG re-adopt it as
     *  it drifts, and a fresh lock coasts for LOCK_LOST_MS while it is missing. */
    public void lockOn(int classId, double txDeg, double tyDeg, double confidence) {
        lockClass = classId;
        lockTx = txDeg;
        lockTy = tyDeg;
        lockAge.reset();
        locked = new Sighting(classId, txDeg, tyDeg, confidence, false, groundRange(tyDeg));
    }

    /** Floor distance camera -> ball from the vertical angle. Big number if at/above the horizon. */
    public double groundRange(double tyDeg) {
        double depression = HiveConfig.CAM_PITCH_DEG - tyDeg;   // angle below horizontal to the ball
        if (depression < 1.0) return 999.0;
        return (HiveConfig.CAM_H - HiveConfig.BALL_H) / Math.tan(Math.toRadians(depression));
    }

    /* ---------------- selection / lock internals ---------------- */

    private Sighting compute() {
        List<RawDet> dets = source.stalenessMs() > HiveConfig.MAX_STALENESS_MS ? null : source.latest();

        if (dets == null || dets.isEmpty()) {
            return coastOrDrop();
        }

        RawDet nearest = null;        // fallback: lowest ty = closest on floor
        RawDet lockMatch = null;      // best angular match to the locked ball
        double bestJump = Double.MAX_VALUE;

        for (RawDet d : dets) {
            if (!allowed.contains(d.classId)) continue;
            if (lockClass != null && d.classId != lockClass) continue;   // lock is color-bound
            if (d.confidence < HiveConfig.MIN_CONF) continue;

            if (nearest == null || d.tyDeg < nearest.tyDeg) {
                nearest = d;
            }
            if (lockTx != null) {
                double jump = Math.hypot(d.txDeg - lockTx, d.tyDeg - lockTy);
                if (jump < bestJump) {
                    bestJump = jump;
                    lockMatch = d;
                }
            }
        }

        if (nearest == null) return coastOrDrop();       // nothing confident this frame

        if (lockTx != null && lockMatch != null && bestJump <= HiveConfig.LOCK_GATE_DEG
                && lockAge.milliseconds() <= HiveConfig.LOCK_LOST_MS) {
            return adopt(lockMatch);                     // same ball as last frame, still fresh: keep it
        }
        if (locked != null && lockAge.milliseconds() <= HiveConfig.LOCK_LOST_MS) {
            return lockedPredicted();                    // fresh lock, ball missing: hold, don't flip
        }
        if (effConfirmFrames() > 1) {
            return confirmStep(dets);                    // confirmation armed: N-frame gate first
        }
        return adopt(nearest);                           // no lock, or stale: take the nearest
    }

    /** Confirmation gate that runs only on a fresh (unlocked) target: adopt a
     *  ball only once the SAME ball — same class, within the gate of where it
     *  was the frame before — has been seen `effConfirmFrames()` CONSECUTIVE
     *  DISTINCT camera frames. Returns null while counting (callers hold off),
     *  and a real adopted sighting on the frame the count reaches N, which then
     *  flows into the normal target lock. Only a NEW DetectionSource frame()
     *  advances the count — re-polling the same frame is not a fresh sample. Any
     *  disappearance, class change, or jump beyond the gate restarts the count. */
    private Sighting confirmStep(List<RawDet> dets) {
        long f = source.frame();
        if (f == lastConfirmFrame) return null;          // same camera frame re-polled: not a new sample
        lastConfirmFrame = f;
        RawDet match = null;
        if (confirmClass != null) {
            double gate = effConfirmGate();
            double bestJump = Double.MAX_VALUE;
            for (RawDet d : dets) {
                if (!allowed.contains(d.classId) || d.classId != confirmClass) continue;
                if (d.confidence < HiveConfig.MIN_CONF) continue;
                double jump = Math.hypot(d.txDeg - confirmTx, d.tyDeg - confirmTy);
                if (jump < bestJump) { bestJump = jump; match = d; }
            }
            if (match == null || bestJump > gate) {
                resetCandidate();                        // vanished or jumped: restart the count
                return null;
            }
            confirmTx = match.txDeg;
            confirmTy = match.tyDeg;
            confirmStreak++;
            if (confirmStreak >= effConfirmFrames()) {
                confirmClass = null;                     // candidate promoted to the lock; keep streak
                confirmTx = null;
                confirmTy = null;
                return adopt(match);
            }
            return null;                                 // still confirming
        }
        for (RawDet d : dets) {                          // first frame: seed on the nearest confident detection
            if (!allowed.contains(d.classId)) continue;
            if (d.confidence < HiveConfig.MIN_CONF) continue;
            if (match == null || d.tyDeg < match.tyDeg) match = d;
        }
        if (match == null) return null;
        confirmClass = match.classId;
        confirmTx = match.txDeg;
        confirmTy = match.tyDeg;
        confirmStreak = 1;
        return null;
    }

    /** Frames required before adopting: a per-instance explicit OFF (-1) wins
     *  over everything; otherwise the per-instance arm, else HiveConfig. */
    private int effConfirmFrames() {
        if (confirmFrames < 0) return 0;                 // >= 1 always "off" was impossible before: now it is
        return confirmFrames > 0 ? confirmFrames : HiveConfig.CONFIRM_FRAMES;
    }

    private double effConfirmGate() {
        return confirmGateDeg > 0 ? confirmGateDeg : HiveConfig.CONFIRM_GATE_DEG;
    }

    private void resetCandidate() {
        confirmClass = null;
        confirmTx = null;
        confirmTy = null;
        confirmStreak = 0;
    }

    /** Lock onto a real detection and remember its angles + age. */
    private Sighting adopt(RawDet d) {
        lockClass = d.classId;
        lockTx = d.txDeg;
        lockTy = d.tyDeg;
        lockAge.reset();
        locked = new Sighting(d.classId, lockTx, lockTy, d.confidence,
                false, groundRange(lockTy));
        return locked;
    }

    /** Re-issue the locked ball's last-known angles while it is briefly missing. */
    private Sighting lockedPredicted() {
        if (locked == null) return null;
        return new Sighting(locked.classId, locked.txDeg, locked.tyDeg, locked.confidence,
                true, locked.distIn);
    }

    private Sighting coastOrDrop() {
        if (locked != null && lockAge.milliseconds() <= HiveConfig.LOCK_LOST_MS) {
            return lockedPredicted();
        }
        dropLock();
        return null;
    }

    private void dropLock() {
        lockClass = null;
        lockTx = null;
        lockTy = null;
        locked = null;
        resetCandidate();
    }

    private static Set<Integer> setOf(Integer... ids) {
        return new HashSet<>(Arrays.asList(ids));
    }
}