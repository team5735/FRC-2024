package frc.robot.constants;

/**
 * Constants file for storing public static final data,
 * hence constants. This file is specifically intended to be used in conjunction
 * with {@link frc.robot.subsystems.shooter.ShooterTopSubsystem} and {@link frc.robot.subsystems.shooter.ShooterBottomSubsystem}.
 */
public class ShooterConstants {
    // The default shooting setpoint for top and bottom, in rotations per minute
    public static final double TOP_DEFAULT_RPM = 3500.0;
    public static final double BOTTOM_DEFAULT_RPM = 2250.0;

    // The stage back shooting setpoint for top and bottom, in rotations per minute
    public static final double TOP_STAGE_BACK_RPM = 4000.0;
    public static final double BOTTOM_STAGE_BACK_RPM = 4000.0;

    // The stage front shooting setpoint for top and bottom, in rotations per minute
    public static final double TOP_STAGE_FRONT_RPM = 3500.0;
    public static final double BOTTOM_STAGE_FRONT_RPM = 3500.0;

    // PID values for top shooter feedback
    public static final double TOP_KP = 0.00195;
    public static final double TOP_KI = 0;
    public static final double TOP_KD = 0;

    // SV values for top shooter feedforward
    public static final double TOP_KS = 0;
    public static final double TOP_KV = 0.001925;

    // PID values for bottom shooter feedback
    public static final double BOTTOM_KP = 0.00195;
    public static final double BOTTOM_KI = 0;
    public static final double BOTTOM_KD = 0;

    // SV values for bottom shooter feedforward
    public static final double BOTTOM_KS = 0;
    public static final double BOTTOM_KV = 0.00194;
}
