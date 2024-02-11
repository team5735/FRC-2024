package frc.robot.constants;

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

    // BS approximated values from testing
    public static final Translation3d HOOD_POS = new Translation3d(5.7 + 2.8, 1.3 + 0.72, 0.0);
    // meters
    public static final double BOT_SHOOTING_DISTANCE = 69.0;
}
