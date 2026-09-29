/*
 * HiveConfig - ONE block to edit when you run this library on your robot.
 *
 * Every driver (BallTracker, BallChaseController, BallChaseFollower,
 * BallWrangler / MecanumWrangler) sources its defaults from here at class
 * load, so retuning the chase means editing this file, not hunting through
 * classes. Settings are deliberately NOT final: tweak at the top and
 * re-flash.
 *
 * Edit this block for your robot:
 *   - wiring names (motors + Limelight)
 *   - camera mount (height, pitch, offsets, FOV)
 *   - detection gating (confidence floor, staleness, lock behavior)
 *   - chase / search / pickup tuning
 *   - Pedro planning and wrapper verb knobs
 *
 * What is NOT here: class ids are tied to your trained model's label list
 * (set them below), and ball geometry (BALL_H, ball size) is a game truth.
 */
package org.firstinspires.ftc.teamcode;

public final class HiveConfig {

    private HiveConfig() {
    }

    /* ---------------- robot wiring ---------------- */

    public static String LIMELIGHT_NAME = "limelight";       // your Limelight 3A's hardware name
    public static int    LIMELIGHT_PIPELINE = 0;             // detector pipeline index in the web UI
    public static int    LIMELIGHT_POLL_RATE_HZ = 100;       // how often the SDK reads the camera
    public static String MOTOR_LEFT_FRONT  = "leftFront";
    public static String MOTOR_RIGHT_FRONT = "rightFront";
    public static String MOTOR_LEFT_BACK   = "leftBack";
    public static String MOTOR_RIGHT_BACK  = "rightBack";
    public static String MOTOR_INTAKE      = "intake";        // "" = no intake wired

    /* ---------------- camera geometry (measure these) ---------------- */

    public static double CAM_PITCH_DEG = 25.0;     // camera tilt BELOW horizontal
    public static double CAM_H         = 9.2;      // camera lens height above floor, in
    public static double BALL_H        = 3.0;      // ball center height above floor, in
    public static double CAM_X_OFFSET  = 0.0;      // camera FORWARD of robot center, in
    public static double CAM_Y_OFFSET  = 0.0;      // camera LEFT of robot center, in; negative = right
    public static double HFOV_DEG      = 54.5;     // verify for your unit
    public static double VFOV_DEG      = 42.0;
    public static double FOV_MARGIN_DEG = 4.0;     // ignore frame edges for field projection

    /* ---------------- class ids - CONFIRM against the pipeline label list ---- */

    public static int CLASS_YELLOW_NEUTRAL = 0;
    public static int CLASS_RED            = 1;
    public static int CLASS_BLUE           = 2;

    /* ---------------- detection gating ---------------- */

    public static double MIN_CONF          = 0.44; // detector confidence, 0..1 on Limelight 3A firmware
    public static long   MAX_STALENESS_MS  = 150;  // Limelight SDK staleness in ms ("Is The Data Fresh?").
    // Keep ~3-4x the 3A SSD detector's frame cadence (~15-20 FPS),
    // so a real frame is never dropped just before pickup;
    // verify the actual cadence on your bench via staleness telem.
    public static double LOCK_GATE_DEG     = 12.0; // max frame-to-frame angular jump to count as "same ball"
    public static long   LOCK_LOST_MS      = 300;  // drop the lock if it isn't matched for this long
    public static int    CONFIRM_FRAMES    = 0;    // require the same ball N CONSECUTIVE frames before adopting; 0 = off
    public static double CONFIRM_GATE_DEG  = 12.0; // max angular jump between confirming frames, deg

    /* ---------------- chase tuning (controller + follower + wrapper) -------- */

    public static double STOP_DIST          = 16.0; // camera->ball floor distance at pickup, in
    public static double AIM_TOL_DEG        = 5.0;  // "aimed enough" to pick up
    public static double DRIVE_MIN_TX       = 15.0; // beyond this many degrees off, turn in place only
    public static double DRIVE_KP           = 0.03; // forward power per inch of distance error
    public static double MIN_FWD            = 0.15; // overcome static friction
    public static double MAX_FWD            = 0.7;
    public static double TURN_KP            = 0.025;// power per degree of tx
    public static double MIN_TURN           = 0.08; // friction floor when outside the tolerance
    public static double MAX_TURN           = 0.6;
    public static double SEARCH_TURN        = 0.30; // clockwise scan when nothing is visible
    public static double SEARCH_SWEEP_DEG   = 360.0;// give up (DONE) if we dead-reckon this far with nothing in view
    public static double TURN_RATE_RAD_PER_POWER_SEC = 3.5; // heading estimate for the sweep cap
    public static double COAST_MAX_DIST     = 26.0; // only coast if last seen within this, in
    public static double COAST_POWER        = 0.25;
    public static long   COAST_MS           = 450;
    public static long   CHASE_LOST_MS      = 500;  // no ball this long in chase -> forget it
    public static long   PICKUP_DWELL_MS    = 600;
    public static long   PICKUP_CONFIRM_MS  = 800;  // after the dwell, keep looking for a pickup confirmer
    // this long; <= 0 = check once at the end of the dwell
    public static double INTAKE_POWER       = 1.0;
    public static long   PREDICT_TURN_MS    = 150;  // keep turning toward a predicted (briefly missing) ball
    // this long after its last real sighting, then hold;
    // must stay under LOCK_LOST_MS

    /* ---------------- Pedro planning (follower) ---------------- */

    public static double MAX_PLAN_RANGE      = 40.0;
    public static double MERGE_RADIUS        = 4.0;   // samples this close are the same ball
    public static double FOLLOWER_APPROACH_DIST = 24.0; // Pedro stops this far short of the ball
    public static double CLEAR_RADIUS        = 12.0;  // balls this close to a pickup are cleared
    public static double FIELD_MIN           = 6.0;
    public static double FIELD_MAX           = 138.0;
    public static long   TRAVEL_TIMEOUT_MS   = 4000;
    public static double HUNT_BUDGET_SEC     = 20.0; // whole-hunt default budget: a target that keeps
    // re-appearing but is never reached can't spin forever;
    // 0 = unlimited

    public static long   SETTLE_MS           = 250;
    public static long   SCAN_MS             = 250;
    public static double SEARCH_STEP_DEG     = 45.0;
    public static int    SEARCH_MAX_STEPS    = 8;

    public static double TURN_TO_KP          = 0.8;   // power per radian of error
    public static double TURN_TO_MIN         = 0.05;  // friction floor; low so the robot EASES into the
    // tolerance instead of overshooting the 4 deg stop
    public static double TURN_TO_MAX         = 0.5;
    public static double TURN_TO_TOL_DEG     = 4.0;
    public static long   TURN_TIMEOUT_MS     = 2500;

    /* ---------------- wrapper verbs ---------------- */

    public static double GRAB_TIMEOUT_SEC    = 6.0;   // per-ball chase before a step gives up
    public static int    GRAB_FAIL_LIMIT     = 2;     // consecutive failed chases before a gather gives up
    public static double WRAPPER_APPROACH_DIST = 28.0; // camera-relative approach standoff, in
    public static double NUDGE_DIST          = 12.0;
    public static double ALIGN_TOL_DEG       = 4.0;
    public static double ALIGN_TIMEOUT_SEC   = 2.5;
    public static double ALIGN_TURN_KP       = 0.05;
    public static double SCAN_SWEEP_DEG      = 120.0;
    public static double SCAN_TIMEOUT_SEC    = 3.0;
    public static double BACK_POWER          = 0.35;
    public static double MAX_FWD_IN_PER_SEC  = 18.0;
    public static double SCORE_OFF_AXIS_PENALTY = 0.25; // per OFF_AXIS_SCALE_DEG of |tx|
    public static double SCORE_CONF_BONUS    = 0.20;   // up to -20% range for max confidence
    public static double OFF_AXIS_SCALE_DEG  = 45.0;
    public static double GO_TO_TIMEOUT_SEC   = 8.0;    // pose verbs (searchAt / then / thenReturnTo)
    public static double GO_TO_POWER         = 0.7;
    public static long   LOOP_MS             = 10;     // BallWrangler.go() loop period
}