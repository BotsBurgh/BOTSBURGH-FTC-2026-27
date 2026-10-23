//package org.firstinspires.ftc.teamcode.autos;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//// Adjust to wherever your Pedro Constants class lives
//import org.firstinspires.ftc.teamcode.pedro.Constants;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.Blob;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobColor;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobConfig;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobControl;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobMath;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobTelemetry;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobTracker;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.LimelightBlobCamera;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.PedroBlobDrive;
//
///**
// * Tune + verify everything before trusting it in auto.
// *
// * Drive   : left stick = move, right stick X = turn (robot-centric)
// * A       : cycle target color  ANY -> POLLEN -> RED_NECTAR -> BLUE_NECTAR
// * Hold RB : visual-servo chase of the locked artifact (checks your turn/strafe signs!)
// *
// * Calibration: put an artifact exactly KNOWN_DISTANCE_IN from the camera lens along the floor,
// * look at "SUGGEST" lines and copy the numbers into your BlobConfig.
// */
//@TeleOp(name = "Blob Calibration", group = "Vision")
//public class BlobCalibrationTeleOp extends OpMode {
//
//    // Edit me: floor distance from the lens to the artifact you placed for calibration
//    private static final double KNOWN_DISTANCE_IN = 24.0;
//
//    // Edit me: Limelight pipeline index that holds your neural model
//    private static final int MODEL_PIPELINE = 0;
//
//    private Follower follower;
//    private PedroBlobDrive drive;
//    private BlobConfig cfg;
//    private LimelightBlobCamera camera;
//    private BlobTracker tracker;
//
//    private BlobColor[] modes;
//    private int modeIdx = 0;
//    private boolean lastA = false;
//
//    @Override
//    public void init() {
//        follower = Co     nstants.create(hardwareMap);
//        follower.setPose(new Pose(0, 0, 0));
//        drive = new PedroBlobDrive(follower);
//
//        // Put YOUR numbers here (mount geometry matters most)
//        cfg = new BlobConfig()
//                .cameraName("limelight")
//                .detectorPipeline(MODEL_PIPELINE)   // neural mode (your pollen/nectar model)
//                .cameraMount(8.0, 20.0, 6.0, 0.0)
//                .blobCenterHeightIn(2.5);
//
//        camera = new LimelightBlobCamera(hardwareMap, cfg);
//        tracker = new BlobTracker(cfg);
//        modes = new BlobColor[]{BlobColor.ANY, BlobColor.POLLEN, BlobColor.RED_NECTAR, BlobColor.BLUE_NECTAR};
//    }
//
//    @Override
//    public void start() {
//        camera.setTarget(modes[modeIdx]);
//        camera.start();
//    }
//
//    @Override
//    public void loop() {
//        long now = System.nanoTime() / 1_000_000L;
//
//        if (gamepad1.a && !lastA) {
//            modeIdx = (modeIdx + 1) % modes.length;
//            camera.setTarget(modes[modeIdx]);
//            tracker.reset();
//        }
//        lastA = gamepad1.a;
//
//        tracker.update(camera.update(now, drive.pose()), now);
//        Blob t = tracker.target();
//
//        if (gamepad1.right_bumper && t != null && tracker.isConfirmed()) {
//            drive.drive(BlobControl.chase(cfg, t));
//        } else {
//            drive.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
//        }
//        follower.update();
//
//        telemetry.addData("Mode (A)", modes[modeIdx]);
//        BlobTelemetry.addCamera(telemetry, camera);
//        BlobTelemetry.addTarget(telemetry, tracker);
//        BlobTelemetry.addAll(telemetry, tracker, 4);
//
//        if (t != null) {
//            telemetry.addLine("--- calibration (artifact at KNOWN_DISTANCE_IN = " + KNOWN_DISTANCE_IN + ") ---");
//            telemetry.addData("SUGGEST cameraPitchDownDeg", "%.2f",
//                    BlobMath.suggestPitchDownDeg(cfg, t.tyDeg, KNOWN_DISTANCE_IN));
//            telemetry.addData("SUGGEST areaConstantK", "%.2f",
//                    BlobMath.suggestAreaK(t.areaPct, KNOWN_DISTANCE_IN));
//        }
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        camera.stop();
//        drive.stop();
//    }
//}
