package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // 1. Drivetrain Configuration (Run MecanumTuner to populate motor directions)
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");
        c.frontRightName.set("rf");
        c.backLeftName.set("lr");
        c.backRightName.set("rr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // 2. Localizer Configuration (Run PinpointTuner to populate pod types, offsets, and directions)
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(10.545222965751108);
        c.yPodOffset.set(1.1915069489967167);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // 3. Foresight Algorithm Configuration (Run ForesightTuner to populate PID & Feedforward gains)
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.36515824171174527);
                Controller secondaryTranslationalForward = Controller.proportional(0.13491621310457028);
                Controller primaryTranslationalLateral = Controller.proportional(0.6450655975519709);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23833450182523802);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014698613786621411));
                c.brake.set(Controller.proportionalFeedforward(0.012493821718628199));

                c.headingFeedback.set(Controller.proportional(5.371504581492874));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04737311892042993, 0.006080480324937184));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10466480413918691, 0.04434867533042755));
                c.quadraticBrakeCoefficients.set(Matrix.diag(7.595386670097607E-4, 0.001967382858803791));

                c.maxAchievableForwardVelocity.set(58.35580200107914);
                c.maxAchievableStrafeVelocity.set(60.332746899613745);
                c.naturalForwardDeceleration.set(35.65317619976304);
                c.naturalStrafeDeceleration.set(71.9470803222035);
            }
    );

    /**
     * Constructs and returns the Pedro Pathing Follower instance.
     */
    public static Follower create(HardwareMap h) {
        Mecanum drivetrain = new Mecanum(h, drivetrainConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(h, localizerConfig);
        return new Follower(localizer, drivetrain, new Foresight(foresightConfig));
    }
}
