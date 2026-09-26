package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;

public class Constants {

    public static Follower create(HardwareMap h) {

        return null;
    }

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {

        c.frontLeftName.set("lf");
        c.frontRightName.set("rf");
        c.backLeftName.set("lr");
        c.backRightName.set("rr");

        c.frontLeftDirection.set(
                DcMotorSimple.Direction.REVERSE
        );

        c.frontRightDirection.set(
                DcMotorSimple.Direction.FORWARD
        );

        c.backLeftDirection.set(
                DcMotorSimple.Direction.REVERSE
        );

        c.backRightDirection.set(
                DcMotorSimple.Direction.FORWARD
        );
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {

        c.name.set("pinpoint");

        c.podType.set(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        c.xPodOffset.set(9.407303577332986);
        c.yPodOffset.set(0.9646629723976916);

        c.xPodDirection.set(
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        c.yPodDirection.set(
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
}