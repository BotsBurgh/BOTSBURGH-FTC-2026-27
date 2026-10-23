package org.firstinspires.ftc.teamcode.vision2.blobvision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Every tunable number lives here. Fields are public so you can also set them directly;
 * the fluent setters just make it nicer to build in one expression.
 *
 * Robot frame used everywhere: +forward, +left, bearing/turn CCW+ (left = positive).
 */
public class BlobConfig {

    public enum DistanceMode {
        /** Floor-plane projection from tx/ty + camera mount geometry (best, needs mount numbers). */
        ANGLE,
        /** distance = areaConstantK / sqrt(area%). Calibrate with BlobCalibrationTeleOp. */
        AREA
    }

    public enum SelectMode { CLOSEST, LARGEST, CENTERED }

    // ---------------------------------------------------------------- hardware
    public String cameraName = "limelight";
    public int pollRateHz = 100;

    /** Colors scanned when the target color is ANY. One color pipeline per color. */
    public List<BlobColor> colors = new ArrayList<>(Arrays.asList(BlobColor.PURPLE, BlobColor.GREEN));

    /** >= 0 switches to neural-detector mode using this pipeline (class names map to colors). */
    public int detectorPipeline = -1;

    /** ANY mode (color pipelines): how long to sit on one pipeline before flipping to the next. */
    public long anyCycleMs = 80;
    /** ANY mode: how long a color's last sighting stays valid while the camera looks at another color. */
    public long anyHoldMs = 250;

    // ---------------------------------------------------------------- camera mount (inches / degrees)
    public double cameraHeightIn = 8.0;        // lens height above floor
    public double cameraPitchDownDeg = 0.0;   // tilt below horizontal (positive = looking down)
    public double cameraYawDeg = 0.0;          // + = camera rotated left of robot forward
    public double cameraForwardOffsetIn = 6.0; // lens position ahead of robot center
    public double cameraLeftOffsetIn = 0.0;    // lens position left of robot center
    public double blobCenterHeightIn = 2.5;    // height of the artifact's center above the floor

    // ---------------------------------------------------------------- distance + filtering
    public DistanceMode distanceMode = DistanceMode.ANGLE;
    public double areaConstantK = 0.0;         // 0 = area fallback disabled
    public double minAreaPct = 0.05;
    public double maxAreaPct = 60.0;
    public double minConfidence = 0.5;         // neural mode, 0..1
    public long maxStalenessMs = 120;          // drop Limelight results older than this
    public double maxRangeIn = 70.0;           // ignore blobs farther than this from the lens

    // ---------------------------------------------------------------- tracker
    public SelectMode selectMode = SelectMode.CLOSEST;
    public double emaAlpha = 0.6;              // 1 = no smoothing, lower = smoother
    public int confirmFrames = 3;              // loops a target must persist before we chase it
    public long lostTimeoutMs = 300;           // lock survives this long without a match
    public double lockRadiusIn = 8.0;          // max movement between frames to count as same blob

    // ---------------------------------------------------------------- chase / intake
    public double kTurn = 0.018;               // turn power per degree of bearing error
    public double maxTurn = 0.45;
    public double minTurn = 0.06;
    public double aimToleranceDeg = 1.5;
    public double kForward = 0.035;            // forward power per inch of remaining distance
    public double minForward = 0.14;
    public double maxForward = 0.55;
    public double alignSlowDeg = 25.0;         // forward power fades to 0 at this much bearing error
    public double engageDistanceIn = 16.0;     // distance (from robot center) where the intake reaches it
    public double blindDistanceIn = 20.0;      // if lost closer than this, assume it went under the robot
    public double pushPower = 0.5;             // blind forward drive into the intake
    public long pushMs = 450;
    public double searchTurnPower = 0.3;
    public long searchFlipMs = 1200;           // search sweep reverses direction this often

    // ---------------------------------------------------------------- fluent setters
    public BlobConfig cameraName(String v) { cameraName = v; return this; }
    public BlobConfig pollRateHz(int v) { pollRateHz = v; return this; }
    public BlobConfig colors(BlobColor... v) { colors = new ArrayList<>(Arrays.asList(v)); return this; }
    public BlobConfig detectorPipeline(int v) { detectorPipeline = v; return this; }
    public BlobConfig anyCycleMs(long v) { anyCycleMs = v; return this; }

    public BlobConfig cameraMount(double heightIn, double pitchDownDeg, double forwardOffsetIn, double leftOffsetIn) {
        cameraHeightIn = heightIn;
        cameraPitchDownDeg = pitchDownDeg;
        cameraForwardOffsetIn = forwardOffsetIn;
        cameraLeftOffsetIn = leftOffsetIn;
        return this;
    }
    public BlobConfig cameraYawDeg(double v) { cameraYawDeg = v; return this; }
    public BlobConfig blobCenterHeightIn(double v) { blobCenterHeightIn = v; return this; }

    public BlobConfig distanceMode(DistanceMode v) { distanceMode = v; return this; }
    public BlobConfig areaConstantK(double v) { areaConstantK = v; return this; }
    public BlobConfig areaRange(double minPct, double maxPct) { minAreaPct = minPct; maxAreaPct = maxPct; return this; }
    public BlobConfig minConfidence(double v) { minConfidence = v; return this; }
    public BlobConfig maxRangeIn(double v) { maxRangeIn = v; return this; }

    public BlobConfig selectMode(SelectMode v) { selectMode = v; return this; }
    public BlobConfig smoothing(double alpha) { emaAlpha = alpha; return this; }
    public BlobConfig confirmFrames(int v) { confirmFrames = v; return this; }

    public BlobConfig turnGains(double kTurn, double maxTurn) { this.kTurn = kTurn; this.maxTurn = maxTurn; return this; }
    public BlobConfig forwardGains(double kForward, double minForward, double maxForward) {
        this.kForward = kForward;
        this.minForward = minForward;
        this.maxForward = maxForward;
        return this;
    }
    public BlobConfig engageDistanceIn(double v) { engageDistanceIn = v; return this; }
    public BlobConfig blindDistanceIn(double v) { blindDistanceIn = v; return this; }
    public BlobConfig push(double power, long ms) { pushPower = power; pushMs = ms; return this; }
    public BlobConfig search(double turnPower, long flipMs) { searchTurnPower = turnPower; searchFlipMs = flipMs; return this; }
}
