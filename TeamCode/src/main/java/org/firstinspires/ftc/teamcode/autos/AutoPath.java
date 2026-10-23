package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends OpMode {

    private enum PathState {
        PATH1,
        PATH2,
        FINISHED
    }

    private Follower follower;
    private PathState pathState;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(47.1667, 12.5202, 180);

    private final Pose path1End =
            poseFactory.of(15.7457, 14.8563, 180);

    private final Pose point2Start =
            poseFactory.of(15.7457, 14.8563, 180);

    private final Pose point2 =
            poseFactory.of(16.671, 109.2932, 180);

    private final Pose point2Control1 =
            poseFactory.of(45.7126, 50.1853, 180);

    private Path path1;
    private Path path2;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);

        follower.setPose(start);

        path1 = Paths.line(start, path1End)
                .linear(start, path1End);

        path2 = Paths.curve(
                point2Start,
                point2Control1,
                point2
        ).linear(
                point2Start,
                point2
        );

        pathState = PathState.PATH1;

        telemetry.addLine("Auto Ready");
        telemetry.update();
    }

    @Override
    public void start() {

        follower.follow(path1);
    }

    @Override
    public void loop() {

        follower.update();

        switch (pathState) {

            case PATH1:

                if (!follower.isBusy()) {
                    pathState = PathState.PATH2;
                    follower.follow(path2);
                }

                break;

            case PATH2:

                if (!follower.isBusy()) {
                    pathState = PathState.FINISHED;
                }

                break;

            case FINISHED:

                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData(
                "X",
                "%.2f",
                follower.pose().x()
        );
        telemetry.addData(
                "Y",
                "%.2f",
                follower.pose().y()
        );
        telemetry.addData(
                "Heading",
                "%.2f",
                Math.toDegrees(follower.pose().heading())
        );

        if (follower.currentPath() != null) {
            telemetry.addData(
                    "Distance Remaining",
                    "%.2f",
                    follower.distanceToEndpoint()
            );

            telemetry.addData(
                    "Path Number",
                    follower.pathIndex()
            );
        }

        telemetry.update();
    }

    @Override
    public void stop() {

        if (follower != null) {
            follower.stop();
        }
    }
}