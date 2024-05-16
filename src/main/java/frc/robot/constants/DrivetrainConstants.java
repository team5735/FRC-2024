package frc.robot.constants;

public class DrivetrainConstants {
    /**
     * How to limit the drivetrain's acceleration. Should always be DISABLED: the
     * limelight runs off of a different battery now.
     */
    public enum SlewRateLimiterMode {
        DISABLED, THETA_MAGNITUDE, AXES
    };

    public static final double DEADBAND = 0.15;

    /**
     * Acceleration-limiting constants for AXES.
     */
    public static final double ACCEL_LIMIT_AXES = 9.0;

    /**
     * Acceleration-limiting constants for THETA_MAGNITUDE.
     */
    public static final double ACCEL_LIMIT_THETA_MAGNITUDE = 6.0;
    public static final double ACCEL_LIMIT_OMEGA = 6.0;
    public static final SlewRateLimiterMode SLEW_RATE_LIMITER_MODE = SlewRateLimiterMode.DISABLED;

    /**
     * Speed multipliers for the different states.
     */
    public static final double SLOW_SPEED = 2;
    public static final double NORMAL_SPEED = 6;
    public static final double TURBO_SPEED = 10;
}
