package frc.robot.constants;

public class AngleConstants {
    public static final double KP = 0.0625;
    public static final double KI = 0;
    public static final double KD = 0.005;

    public static final double KS = 0;
    public static final double KG = 0.78;
    public static final double KV = 0;

    public static final double BASE_POS_ROT = 235. / 360;
    public static final double BASE_POS_DEG = convertRotationsToDegrees(BASE_POS_ROT);

    public static final double HIGHEST_DEG = 237;
    public static final double LOWEST_DEG = 120;

    public static final double AMP_DEG = 125;
    public static final double STAGE_BACK_DEG = 190;
    public static final double STAGE_FRONT_DEG = 200;

    // planetary 9:1, chain drive 16:42
    public static final double MOTOR_TO_OUTPUT_RATIO = 1. / 9 * 16. / 42;
    // inverse of the previous ratio
    public static final double OUTPUT_TO_MOTOR_RATIO = 1. / MOTOR_TO_OUTPUT_RATIO;

    public static double convertRotationsToDegrees(double rotations) {
        rotations %= 1;
        if (rotations < 0)
            rotations += 1;
        rotations *= 360;

        return rotations;
    }
}
