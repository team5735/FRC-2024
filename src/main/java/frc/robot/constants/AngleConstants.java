package frc.robot.constants;

public class AngleConstants {
    public static final double ANGLE_KP = 0;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0;

    public static final double ANGLE_KS = 0;
    public static final double ANGLE_KG = 0;
    public static final double ANGLE_KV = 0;

    public static double convertTalonRotationsToDegrees(double rotations) {
        rotations %= 1;
        if (rotations < 0)
            rotations += 1;
        rotations *= 360;

        return rotations;
    }
}
