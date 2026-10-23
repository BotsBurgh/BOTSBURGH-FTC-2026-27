package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Adjust to wherever your Pedro Constants class lives
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobCollectStep;
import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobColor;
import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobConfig;
import org.firstinspires.ftc.teamcode.vision2.blobvision.LimelightBlobCamera;
import org.firstinspires.ftc.teamcode.vision2.blobvision.PedroBlobDrive;
import org.firstinspires.ftc.teamcode.vision2.blobvision.StepChain;

/**
 * Shows how to bolt a vision-collect chain onto an existing autonomous state machine.
 *
 * Copy the "BLOB" parts into YOUR auto: the fields, the init() setup, and the COLLECT state.
 * Everything else here is placeholder for your own states.
 */
@Autonomous(name = "Example Auto + Blob Chain", group = "Vision")
public class ExampleBlobAuto extends OpMode {

    private enum S { YOUR_AUTO, COLLECT, FINISH }

    private Follower follower;
    private S state = S.YOUR_AUTO;

    // ---- BLOB ------------------------------------------------------------
    private BlobConfig cfg;
    private LimelightBlobCamera camera;
    private PedroBlobDrive drive;
    private StepChain collectChain;
    // ----------------------------------------------------------------------

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0)); // your start pose

        // ---- BLOB ----
        cfg = new BlobConfig()
                .colors(BlobColor.PURPLE, BlobColor.GREEN)
                .cameraMount(8.0, 20.0, 6.0, 0.0)   // lens height, pitch down, fwd offset, left offset
                .engageDistanceIn(16.0);

        camera = new LimelightBlobCamera(hardwareMap, cfg);
        drive = new PedroBlobDrive(follower);

        collectChain = StepChain.create()
                // purple first: up to 2 artifacts or 6 seconds, whichever comes first
                .then(new BlobCollectStep(camera, drive, cfg)
                        .color(BlobColor.PURPLE)
                        .collect(2)
                        .forSeconds(6)
                        .intake(this::intakeOn, this::intakeOff)
                        // .pickup(PickupDetector.sensor(() -> beamBreak.getState() == false, 80))
                        .telemetry(telemetry, false))
                // then green: 1 artifact or 4 seconds
                .then(new BlobCollectStep(camera, drive, cfg)
                        .color(BlobColor.GREEN)
                        .collect(1)
                        .forSeconds(4)
                        .intake(this::intakeOn, this::intakeOff)
                        .telemetry(telemetry, false));
    }

    @Override
    public void start() {
        camera.start();
    }

    @Override
    public void loop() {
        follower.update(); // must run every loop (PedroBlobDrive relies on it)

        switch (state) {
            case YOUR_AUTO:
                // ... your existing path states live here ...
                if (yourAutoIsDone()) state = S.COLLECT;
                break;

            case COLLECT:
                // ---- BLOB: one line drives the whole chain ----
                if (collectChain.update()) {
                    state = S.FINISH;
                }
                break;

            case FINISH:
                drive.stop();
                break;
        }

        telemetry.addData("Auto state", state);
        telemetry.update();
    }

    @Override
    public void stop() {
        collectChain.stop();
        camera.stop();
        drive.stop();
    }

    private boolean yourAutoIsDone() {
        return true; // replace with e.g. !follower.isBusy() once your path finishes
    }

    private void intakeOn() { /* intakeMotor.setPower(1); */ }
    private void intakeOff() { /* intakeMotor.setPower(0); */ }
}
