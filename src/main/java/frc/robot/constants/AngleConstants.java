package frc.robot.constants;

public class AngleConstants {
    public static final double ANGLE_KP = 0.052;
    public static final double ANGLE_KI = 0.04;
    public static final double ANGLE_KD = 0.018;

    public static final double ANGLE_KS = 0;
    public static final double ANGLE_KG = 0.515;
    public static final double ANGLE_KV = 0;


    public static final double ANGLE_START_POS_ROT = 335./360; // This means 335 degrees (unit of device rotations)
    public static final double ANGLE_START_POS_DEG = convertRotationsToDegrees(ANGLE_START_POS_ROT);

    public static final double ANGLE_HIGHEST_DEG = 338;
    public static final double ANGLE_LOWEST_DEG = 230;

    public static final double ANGLE_MOTOR_TO_OUTPUT_RATIO = 1./9 * 22./42; 
    // planetary 9:1, pulleys 22:42 (they are divided by one another, leading to this inverse ratio above)
    public static final double ANGLE_OUTPUT_TO_MOTOR_RATIO = 1./ANGLE_MOTOR_TO_OUTPUT_RATIO;
    // inverse of the previous ratio


    public static double convertRotationsToDegrees(double rotations) {
        rotations %= 1;
        if (rotations < 0)
            rotations += 1;
        rotations *= 360;

        return rotations;
    }
}
