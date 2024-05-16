// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner
 * classes) wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * Ports for the controllers. Shouldn't really be changed.
     */
    public static class OperatorConstants {
        /**
         * Controller USB port for the primary driver, can be configured in Driver
         * Station
         */
        public static final int DRIVER_CONTROLLER_PORT = 0;
        /**
         * Controller USB port for the secondary driver, "subsystem controller", can be
         * configured in Driver Station
         */
        public static final int SUBSYSTEM_CONTROLLER_PORT = 1;
    }

    /**
     * These are the CAN IDs of most of the subsystems in the robot. Other constants
     * here are either very common across many subsystems or should be sorted.
     */

     
    /** CAN ID for the CTRE CANdle */
    public static final int CANDLE_ID = 33;

    /**
     * CAN ID for the right Talon FX motor controller used by
     * {@link frc.robot.subsystems.AngleSubsystem}
     */
    public static final int ANGLE_MOTOR_RIGHT_ID = 15; // TALON
    /**
     * CAN ID for the left Talon FX motor controller used by
     * {@link frc.robot.subsystems.AngleSubsystem}
     */
    public static final int ANGLE_MOTOR_LEFT_ID = 23; // TALON
    /**
     * DIO pin for the PWM absolute encoder used by
     * {@link frc.robot.subsystems.AngleSubsystem}
     */
    public static final int ANGLE_ENCODER_PIN = 1; // ENCODER (DIO)

    /**
     * CAN ID for the right SparkMax motor controller used by
     * {@link frc.robot.subsystems.ClimberSubsystem}
     */
    public static final int CLIMBER_MOTOR_RIGHT_ID = 31; // SPARKMAX
    /**
     * CAN ID for the left SparkMax motor controller used by
     * {@link frc.robot.subsystems.ClimberSubsystem}
     */
    public static final int CLIMBER_MOTOR_LEFT_ID = 61; // SPARKMAX

    /**
     * CAN ID for the SparkMax motor controller used by
     * {@link frc.robot.subsystems.FeederSubsystem}
     */
    public static final int FEEDER_MOTOR_ID = 30; // SPARKMAX
    /**
     * DIO pin for the beam break sensor used by
     * {@link frc.robot.subsystems.FeederSubsystem}
     */
    public static final int FEEDER_BEAM_PIN = 0; // LIGHT-BASED SENSOR (DIO)

    /**
     * CAN ID for the SparkMax motor controller used by
     * {@link frc.robot.subsystems.IntakeSubsystem}
     */
    public static final int INTAKE_MOTOR_ID = 60; // SPARKMAX
    /**
     * DIO pin for the beam break sensor used by
     * {@link frc.robot.subsystems.IntakeSubsystem}
     */
    public static final int INTAKE_BEAM_PIN = 2; // LIGHT-BASED SENSOR (DIO)

    /**
     * CAN ID for the Talon FX motor controller used by
     * {@link frc.robot.subsystems.shooter.ShooterTopSubsystem}
     */
    public static final int SHOOTER_MOTOR_TOP_ID = 57; // TALON
    /**
     * CAN ID for the Talon FX motor controller used by
     * {@link frc.robot.subsystems.shooter.ShooterBottomSubsystem}
     */
    public static final int SHOOTER_MOTOR_BOTTOM_ID = 58; // TALON

    /** CAN ID for the Power Distribution Hub of the overall robot. */
    public static final int PDH_ID = 59;

    /**
     * This tolerance can be used as a starting point for most PIDs on the robot.
     */
    public static final double TOLERANCE = 0.01;
}
