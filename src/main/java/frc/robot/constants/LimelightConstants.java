package frc.robot.constants;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;

public class LimelightConstants {
    // limelight units are in meters
    public static final Translation3d HOOD_POS = new Translation3d(8.269, 1.443, 1.983);
    // meters, corroborate and put correct value please
    public static final double BOT_SHOOTING_DISTANCE = 69.0;
    // meters/second
    public static final double DRIVETRAIN_MOVEMENT_SPEED = 1;
    public static final double CLUELESS_TURN_SPEED = 1;
    // TODO
    public static final Translation3d ANGLE_CHANGER_OFFSET = new Translation3d(0, 0, 0.635);
    public static final double ANGLE_CHANGER_RADIUS = 0;

    public static final double AIMED_ON_TIMER = 1;

    public static final boolean INFINITE_AIM = false;
    public static final PIDConstants ROTATION_PID = new PIDConstants(2, .2, .15);
    public static final double TURN_P = 6;
    public static final double TURN_I = 2;
    public static final double TURN_D = 0;
    public static final double TURN_SRL = 8;

    public static final double BLINK_TIME = 1.0;
}
