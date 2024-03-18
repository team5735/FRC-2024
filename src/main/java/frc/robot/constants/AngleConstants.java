package frc.robot.constants;

public class AngleConstants {
    public static final double ANGLE_KP = 0.0625;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0.005;

    public static final double ANGLE_KS = 0;
    public static final double ANGLE_KG = 0.78;
    public static final double ANGLE_KV = 0;

    public static final double ANGLE_START_POS_ROT = 235. / 360; 
    public static final double ANGLE_START_POS_DEG = convertRotationsToDegrees(ANGLE_START_POS_ROT);

    public static final double ANGLE_HIGHEST_DEG = 237;
    public static final double ANGLE_LOWEST_DEG = 120;

    public static final double ANGLE_AMP_DEG = 125;
    public static final double ANGLE_STAGE_BACK_SHOOT_DEG = 190;
    public static final double ANGLE_STAGE_FRONT_SHOOT_DEG = 200;

    // planetary 9:1, chain drive 16:42
    public static final double ANGLE_MOTOR_TO_OUTPUT_RATIO = 1. / 9 * 16. / 42;
    // inverse of the previous ratio
    public static final double ANGLE_OUTPUT_TO_MOTOR_RATIO = 1. / ANGLE_MOTOR_TO_OUTPUT_RATIO;

    public static double convertRotationsToDegrees(double rotations) {
        rotations %= 1;
        if (rotations < 0)
            rotations += 1;
        rotations *= 360;

        return rotations;
    }
}
