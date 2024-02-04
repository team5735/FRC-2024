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
    public static final int kDriverControllerPort = 0;
  }

  public static int SHOOTER_RIGHT_ID = 55;
  public static int SHOOTER_LEFT_ID = 53;

  public static int CLIMBER_RIGHT_ID = 30;
  public static int CLIMBER_LEFT_ID = 31;

  public static int INTAKE_PULL_ID = 58;

  public static double INTAKE_PULL_SPEED = 4.5;
  public static double INTAKE_PUSH_SPEED = 4.5;

  public static double INTAKE_UP_POS = 270;
  public static double INTAKE_DOWN_POS = 90;

  public static double SHOOTER_SPEED = 1.0;

  public static double CLIMBER_SPEED_RIGHT_UP = 1.0;
  public static double CLIMBER_SPEED_RIGHT_DOWN = 1.0;
  public static double CLIMBER_SPEED_LEFT_UP = 1.0;
  public static double CLIMBER_SPEED_LEFT_DOWN = 1.0;




  public static double convertTalonRotationsToDegrees(double rotations){
    rotations %= 1;
    if(rotations < 0)
      rotations += 1;
    rotations *= 360;
 
    return rotations;
  }
}
