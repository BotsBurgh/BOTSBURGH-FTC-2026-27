//package org.firstinspires.ftc.teamcode.autos;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.pedro.Constants;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobCollectStep;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobColor;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobConfig;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.LimelightBlobCamera;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.PedroBlobDrive;
//import org.firstinspires.ftc.teamcode.vision2.blobvision.StepChain;
//
///**
// * Example: collect game pieces with your pollen / red nectar / blue nectar neural model.
// * Copy the "BLOB" parts into your real auto.
// */
//@Autonomous(name = "Biobuzz Collect Example", group = "Vision")
//public class BiobuzzCollectAuto extends OpMode {
//
//    private enum S { YOUR_AUTO, COLLECT, FINISH }
//
//    // The Limelight pipeline index that holds your uploaded neural model
//    private static final int MODEL_PIPELINE = 0;
//
//    private Follower follower;
//    private S state = S.YOUR_AUTO;
//
//    // ---- BLOB ----
//    private BlobConfig cfg;
//    private LimelightBlobCamera camera;
//    private PedroBlobDrive drive;
//    private StepChain collectChain;
//
//    @Override
//    public void init() {
//        follower = Constants.create(hardwareMap);
//        follower.setPose(new Pose(72, 72, 0)); // your start pose
//
//        // ---- BLOB ----
//        cfg = new BlobConfig()
//                .detectorPipeline(MODEL_PIPELINE)   // neural mode: uses your model
//                .minConfidence(0.5)
//                .cameraMount(8.0, 20.0, 6.0, 0.0)   // MEASURE: lens height, pitch down, fwd offset, left offset
//                .blobCenterHeightIn(2.5)            // MEASURE: height of a piece's center above the floor
//                .engageDistanceIn(16.0);
//
//        camera = new LimelightBlobCamera(hardwareMap, cfg);
//        drive = new PedroBlobDrive(follower);
//
//        collectChain = StepChain.create()
//                // red nectar first: up to 2 or 6 seconds, whichever comes first
//                .then(new BlobCollectStep(camera, drive, cfg)
//                        .color(BlobColor.RED_NECTAR)
//                        .collect(2)
//                        .forSeconds(6)
//                        .intake(this::intakeOn, this::intakeOff)
//                        .telemetry(telemetry, false))
//                // then pollen: 1 piece or 4 seconds
//                .then(new BlobCollectStep(camera, drive, cfg)
//                        .color(BlobColor.POLLEN)
//                        .collect(1)
//                        .forSeconds(4)
//                        .intake(this::intakeOn, this::intakeOff)
//                        .telemetry(telemetry, false));
//        // Use BlobColor.ANY to chase whichever of the three is closest.
//    }
//
//    @Override
//    public void start() {
//        camera.start();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//
//        switch (state) {
//            case YOUR_AUTO:
//                // ... your existing path states ...
//                if (yourAutoIsDone()) state = S.COLLECT;
//                break;
//
//            case COLLECT:
//                if (collectChain.update()) state = S.FINISH;
//                break;
//
//            case FINISH:
//                drive.stop();
//                break;
//        }
//
//        telemetry.addData("Auto state", state);
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        collectChain.stop();
//        camera.stop();
//        drive.stop();
//    }
//
//    private boolean yourAutoIsDone() {
//        return true; // e.g. !follower.isBusy()
//    }
//
//    private void intakeOn() { /* intake.setPower(1); */ }
//    private void intakeOff() { /* intake.setPower(0); */ }
//}
