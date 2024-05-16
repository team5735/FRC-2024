package frc.robot.constants;

/**
 * Constants file for storing public static final data and some utility methods,
 * hence constants. This file is specifically intended to be used in conjunction
 * with {@link frc.robot.subsystems.AngleSubsystem}.
 */
public class AngleConstants {

    // PID values for Angle feedback
    public static final double KP = 0.0625;
    public static final double KI = 0;
    public static final double KD = 0.005;

    // SGV values for Angle feedforward
    public static final double KS = 0;
    public static final double KG = 0.78;
    public static final double KV = 0;

    // The starting position of the structure, in encoder rotation fractions
    public static final double BASE_POS_ROT = 235. / 360;
    // The starting position of the structure, in degrees
    public static final double BASE_POS_DEG = convertRotationsToDegrees(BASE_POS_ROT);

    // The highest safe position of the structure, in degrees
    public static final double HIGHEST_DEG = 237;
    // The lowest safe position of the structure, in degrees
    public static final double LOWEST_DEG = 120;

    // The preset position of the structure for amp shooting, in degrees
    public static final double AMP_DEG = 125;
    // The preset position of the structure for speaker shots from stage back, in
    // degrees
    // This position unfortunately has not been properly tested and is not ideal
    public static final double STAGE_BACK_DEG = 190;
    public static final double STAGE_FRONT_DEG = 200;

    // The rotation ratio from the motor to the structure, calculated thus:
    // planetary 9:1, chain drive 16:42
    public static final double MOTOR_TO_OUTPUT_RATIO = 1. / 9 * 16. / 42;
    // The rotation ratio from the structure from the motor, calculated from the
    // inverse of the previous ratio
    public static final double OUTPUT_TO_MOTOR_RATIO = 1. / MOTOR_TO_OUTPUT_RATIO;

    /**
     * Public static utility method to quickly convert the encoder's rotation
     * fraction readings into more easily readable degrees.
     * 
     * @param rotations - the measurement in device rotation fractions
     * @return the rotations converted into degrees
     */
    public static double convertRotationsToDegrees(double rotations) {
        rotations %= 1;
        if (rotations < 0)
            rotations += 1;
        rotations *= 360;

        return rotations;
    }
}
