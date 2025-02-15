package frc.robot.constants;

public class DrivetrainConstants {
    public enum SlewRateLimiterMode {
        DISABLED, THETA_MAGNITUDE, AXES
    };

    public static final double DEADBAND = 0.15;
    public static final double ACCEL_LIMIT_AXES = 9.0;
    public static final double ACCEL_LIMIT_THETA_MAGNITUDE = 6.0;
    public static final double ACCEL_LIMIT_OMEGA = 6.0;
    public static final SlewRateLimiterMode SLEW_RATE_LIMITER_MODE = SlewRateLimiterMode.DISABLED;

    public static final double SLOW_SPEED = 0.5;
    public static final double NORMAL_SPEED = 2;
    public static final double TURBO_SPEED = 2;

    public static final double MAX_SPEED = 7;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
}
