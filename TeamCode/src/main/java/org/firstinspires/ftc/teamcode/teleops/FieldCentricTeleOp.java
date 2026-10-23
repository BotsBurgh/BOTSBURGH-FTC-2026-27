package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

import java.util.List;

/**
 * FieldCentricTeleOp - True Field-Centric Mecanum Drive.
 *
 * Controls:
 *   - Left Stick Y : Field Forward / Backward (independent of robot heading)
 *   - Left Stick X : Field Strafe Left / Right (independent of robot heading)
 *   - Right Stick X: Robot Rotation (Clockwise / Counter-Clockwise)
 *   - Options / Back / Start : Reset IMU Heading Zero
 *   - Left Trigger : Precision / Slow Mode (0.4x speed)
 */
@TeleOp(name = "Field Centric Drive", group = "Drive")
public class FieldCentricTeleOp extends OpMode {

    private Drivetrain drive;
    private IMU imu;
    private LynxModule[] lynxModules;

    private static final double SLOW_MODE_SCALE = 0.4;

    // Control Hub Mounting Orientation - Adjust to match physical Hub placement
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    @Override
    public void init() {
        // Enable Manual Bulk Caching across REV Hubs
        List<LynxModule> moduleList = hardwareMap.getAll(LynxModule.class);
        lynxModules = moduleList.toArray(new LynxModule[0]);
        for (LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drive = new Drivetrain(hardwareMap);

        // Initialize IMU with orientation
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)));
        imu.resetYaw();
    }

    @Override
    public void loop() {
        // Clear Lynx Bulk Cache once per loop
        for (LynxModule module : lynxModules) {
            module.clearBulkCache();
        }

        // Reset field heading zero when driver presses options/back/start
        if (gamepad1.options || gamepad1.back || gamepad1.start) {
            imu.resetYaw();
        }

        // Current robot yaw heading in radians
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field-centric joystick inputs
        double y = -gamepad1.left_stick_y; // Field Forward (+Y)
        double x = gamepad1.left_stick_x;  // Field Right (+X)
        double turn = gamepad1.right_stick_x; // Rotation

        // Rotate stick vector by -heading to calculate robot-relative powers
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Speed multiplier
        double scale = gamepad1.left_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;

        // Apply robot-relative drive commands
        drive.drive(rotY * scale, rotX * scale, turn * scale);

        // Telemetry
        telemetry.addData("Heading (Deg)", "%.1f°", Math.toDegrees(heading));
        telemetry.addData("Slow Mode", scale < 1.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
