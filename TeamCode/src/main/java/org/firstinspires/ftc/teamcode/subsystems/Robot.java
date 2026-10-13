package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.turret.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.vision.BallChaseController;

/**
 * Owns every subsystem. OpModes create one Robot and use its fields.
 *
 * Usage in an OpMode:
 *   robot = new Robot(hardwareMap);      // init()
 *   robot.update();                      // top of loop()
 *   robot.drive.drive(fwd, turn);
 *   robot.stop();                        // stop()
 */
public class Robot {

    // Temporary test drivetrain, replace with Pedro's Follower when tuned
//    public final Drivetrain drive;
//    public final Intake intake;
    public final Shooter shooter;
//    public final BallChaseController vision;

    public Robot(HardwareMap hardwareMap) {
//        drive = new Drivetrain(hardwareMap);
//        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
//        vision = new BallChaseController(hardwareMap);
    }

    /** Call once at the top of every loop(). */
    public void update() {
//        vision.update();
        // add intake.update(), shooter.update() here if they have them
        shooter.runCR();
        shooter.runTurret();
    }

    /** Call from stop(). */
    public void stop() {
//        drive.stop();
//        vision.stop();
//        intake.stop();
        shooter.stop();
        // add intake/shooter stop methods here if they have them
    }
}