
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.turret.Shooter;

public class Robot {

    public final Shooter shooter;

    public Robot(HardwareMap hardwareMap) {
        shooter = new Shooter(hardwareMap);
    }

    public void update() {
        shooter.runTurret();
    }

    public void stop() {
        shooter.stop();
    }
}
