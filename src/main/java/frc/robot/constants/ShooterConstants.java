package frc.robot.constants;

/**
 * Constants file for storing public static final data,
 * hence constants. This file is specifically intended to be used in conjunction
 * with {@link frc.robot.subsystems.shooter.ShooterTopSubsystem} and
 * {@link frc.robot.subsystems.shooter.ShooterBottomSubsystem}.
 */
public class ShooterConstants {
    /**
     * The default shooting setpoint for Shooter Top , in rotations per minute
     */
    public static final double TOP_DEFAULT_RPM = 3500.0;
    /**
     * The default shooting setpoint for Shooter Bottom , in rotations per minute
     */
    public static final double BOTTOM_DEFAULT_RPM = 2250.0;

    /**
     * The stage back shooting setpoint for Shooter Top, in rotations per minute
     * 
     * @deprecated - this position unfortunately has not been properly tested and
     *             is
     *             not ideal
     */
    public static final double TOP_STAGE_BACK_RPM = 4000.0;
    /**
     * The stage back shooting setpoint for Shooter Bottom, in rotations per minute
     * 
     * @deprecated - this position unfortunately has not been properly tested and
     *             is
     *             not ideal
     */
    public static final double BOTTOM_STAGE_BACK_RPM = 4000.0;

    /**
     * The stage front shooting setpoint for Shooter Top, in rotations per minute
     * 
     * @deprecated - this position unfortunately has not been properly tested and
     *             is
     *             not ideal
     */    
    public static final double TOP_STAGE_FRONT_RPM = 3500.0;
    /**
     * The stage front shooting setpoint for Shooter Bottom, in rotations per minute
     * 
     * @deprecated - this position unfortunately has not been properly tested and
     *             is
     *             not ideal
     */    
    public static final double BOTTOM_STAGE_FRONT_RPM = 3500.0;

    /** P value for Shooter Top feedback PID loop */
    public static final double TOP_KP = 0.00195;
    /** I value for Shooter Top feedback PID loop */
    public static final double TOP_KI = 0;
    /** D value for Shooter Top feedback PID loop */
    public static final double TOP_KD = 0;

    /** S value for Shooter Top feedforward */
    public static final double TOP_KS = 0;
    /** V value for Shooter Top feedforward */
    public static final double TOP_KV = 0.001925;

    /** P value for Shooter Bottom feedback PID loop */
    public static final double BOTTOM_KP = 0.00195;
    /** I value for Shooter Bottom feedback PID loop */
    public static final double BOTTOM_KI = 0;
    /** D value for Shooter Bottom feedback PID loop */
    public static final double BOTTOM_KD = 0;

    /** S value for Shooter Top feedforward */
    public static final double BOTTOM_KS = 0;
    /** V value for Shooter Top feedforward */
    public static final double BOTTOM_KV = 0.00194;
}
