
package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConfig {
    public static boolean ENABLED = false;

    public static double TARGET_VELOCITY = 200.0;

    public static double TURRET_P = 100.0;
    public static double TURRET_I = 0.0;
    public static double TURRET_D = 0.0;
    public static double TURRET_F = 15.3;

    public static double ROLLER_TURRET_RATIO = 1.5;
}
