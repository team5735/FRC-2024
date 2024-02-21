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
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    // ALL IDS ****MUST**** BE RECALIBRATED
    public static final int CANDLE_ID = 33;

    public static final int ANGLE_MOTOR_RIGHT_ID = 20; // TALON
    public static final int ANGLE_MOTOR_LEFT_ID = 21; // TALON

    public static final int CLIMBER_MOTOR_RIGHT_ID = 31; // SPARKMAX
    public static final int CLIMBER_MOTOR_LEFT_ID = 30; // SPARKMAX

    public static final int FEEDER_MOTOR_ID = 40; // SPARKMAX
    public static final int FEEDER_SWITCH_CHANNEL = 0; // LIGHT-BASED SENSOR

    public static final int INTAKE_MOTOR_ID = 60; // SPARKMAX

    public static final int SHOOTER_MOTOR_TOP_ID = 57; // TALON
    public static final int SHOOTER_MOTOR_BOTTOM_ID = 58; // TALON
}
