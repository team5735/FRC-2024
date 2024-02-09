// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public class ShooterConstants {
        public static final int SHOOTER_MOTOR_RIGHT_ID = 55; // talon
        public static final int SHOOTER_MOTOR_LEFT_ID = 53; // talon

        public static final double SHOOTER_RIGHT_VOLTS = 1.0;
        public static final double SHOOTER_LEFT_VOLTS = 1.0;
    }

    public class AngleConstants {
        public static final int ANGLE_MOTOR_RIGHT_ID = 81; // talon
        public static final int ANGLE_MOTOR_LEFT_ID = 82; // talon

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

    public class ClimberConstants {
        public static final int CLIMBER_MOTOR_RIGHT_ID = 30; // sparkmax
        public static final int CLIMBER_MOTOR_LEFT_ID = 31; // sparkmax

        public static final double CLIMBER_RIGHT_UP_VOLTS = 1.0;
        public static final double CLIMBER_RIGHT_DOWN_VOLTS = 1.0;
        public static final double CLIMBER_LEFT_UP_VOLTS = 1.0;
        public static final double CLIMBER_LEFT_DOWN_VOLTS = 1.0;
    }

    public class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 58; // talon

        public static final double INTAKE_PULL_VOLTS = 4.5;
        public static final double INTAKE_PUSH_VOLTS = 4.5;
    }

    public class LimelightConstants {
        // all wrong
        // limelight units are in meters
        public static final int SPEAKER_LEFT_TAG_FIDUCIALID = 1;
        // 1' 8 7/8" = 20.875"
        public static final double SPEAKER_LEFT_TAG_HEIGHT = 0.5302;
        // 15"
        public static final double PIVOT_OFFSET_X = 0.381;
        // 4" (*technically* "3 2/3")
        public static final double PIVOT_OFFSET_Z = 0.1016;
    }

    // ALL IDS ****MUST**** BE RECALIBRATED
    public static final int CANDLE_ID = 69;

    public static final int FEEDER_MOTOR_ID = 78; // sparkmax
}
