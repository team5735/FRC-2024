package frc.robot.constants;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;

public class LimelightConstants {
    // all wrong
    // limelight units are in meters
    public static final int SPEAKER_LEFT_TAG_FIDUCIALID = 1;
    // 1' 8 7/8" = 20.875"
    public static final double SPEAKER_LEFT_TAG_HEIGHT = 0.5302;
    // 15"
    public static final double PIVOT_OFFSET_X = 0.381;
    // 4" (*technically* "3 2/3")
    public static final double PIVOT_OFFSET_Z = 0.1016;

    public static final Translation3d HOOD_POS = new Translation3d(8.269, 1.443, 1.983);
    // meters, corroborate and put correct value please
    public static final double BOT_SHOOTING_DISTANCE = 69.0;
    // meters/second
    public static final double DRIVETRAIN_MOVEMENT_SPEED = 1;
    // TODO
    public static final Translation3d ANGLE_CHANGER_POS = new Translation3d();
    public static final double ANGLE_CHANGER_RADIUS = 0;

    public static final boolean INFINITE_AIM = true;
    public static final PIDConstants ROTATION_PID = new PIDConstants(2, .2, .15);
    public static final double TURN_P = 4;
    public static final double TURN_I = 1.5;
    public static final double TURN_D = 0;
    public static final double TURN_SRL = 2;
}
