package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.SuperSwerveModuleConstantsFactory;
import frc.robot.subsystems.DrivetrainSubsystem;

// Drivetrain dimension wheel to wheel: 20.5 by 25

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // MALISH UPDATE: made these public to use with pathplanner
    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(20).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(.05).withKI(0).withKD(0)
            .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 6;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "squab";
    private static final int kPigeonId = 13;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    // hahaha
    private static final SuperSwerveModuleConstantsFactory ConstantCreator = (SuperSwerveModuleConstantsFactory) (new SuperSwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed));

    // Front Left
    private static final int kFrontLeftDriveMotorId = 4;
    private static final int kFrontLeftSteerMotorId = 3;
    private static final int kFrontLeftEncoderId = 10;
    private static final double kFrontLeftEncoderOffset = -0.149170;

    private static final double kFrontLeftXPosInches = 10.25;
    private static final double kFrontLeftYPosInches = 12.5;

    public static final Slot0Configs kFrontLeftSteerGains = new Slot0Configs()
            .withKP(95).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);

    public static final Slot0Configs kFrontLeftDriveGains = new Slot0Configs()
            .withKP(.05).withKI(0).withKD(0.00)
            .withKS(0).withKV(0).withKA(0);

    // Front Right
    private static final int kFrontRightDriveMotorId = 6;
    private static final int kFrontRightSteerMotorId = 5;
    private static final int kFrontRightEncoderId = 11;
    private static final double kFrontRightEncoderOffset = 0.056885;

    private static final double kFrontRightXPosInches = 9.75;
    private static final double kFrontRightYPosInches = -12.5;

    public static final Slot0Configs kFrontRightSteerGains = new Slot0Configs()
            .withKP(95).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);

    public static final Slot0Configs kFrontRightDriveGains = new Slot0Configs()
            .withKP(.055).withKI(0).withKD(0.00)
            .withKS(0).withKV(0).withKA(0);

    // Back Left
    private static final int kBackLeftDriveMotorId = 2;
    private static final int kBackLeftSteerMotorId = 1;
    private static final int kBackLeftEncoderId = 9;
    private static final double kBackLeftEncoderOffset = -0.490234;

    private static final double kBackLeftXPosInches = -10.25;
    private static final double kBackLeftYPosInches = 12.5;

    public static final Slot0Configs kBackLeftSteerGains = new Slot0Configs()
            .withKP(95).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);

    public static final Slot0Configs kBackLeftDriveGains = new Slot0Configs()
            .withKP(.05).withKI(0).withKD(0.00)
            .withKS(0).withKV(0).withKA(0);

    // Back Right
    private static final int kBackRightDriveMotorId = 8;
    private static final int kBackRightSteerMotorId = 7;
    private static final int kBackRightEncoderId = 12;
    private static final double kBackRightEncoderOffset = 0.079834;

    private static final double kBackRightXPosInches = -10.25;
    private static final double kBackRightYPosInches = -12.5;

    public static final Slot0Configs kBackRightSteerGains = new Slot0Configs()
            .withKP(95).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);

    public static final Slot0Configs kBackRightDriveGains = new Slot0Configs()
            .withKP(.055).withKI(0).withKD(0.00)
            .withKS(0).withKV(0).withKA(0);

    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
            kFrontLeftDriveGains, kFrontRightSteerGains, kFrontLeftEncoderOffset,
            Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
            kFrontRightDriveGains, kFrontRightSteerGains, kFrontRightEncoderOffset,
            Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches),
            kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
            kBackLeftDriveGains, kBackLeftSteerGains, kBackLeftEncoderOffset,
            Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
            kBackRightDriveGains, kBackRightSteerGains, kBackRightEncoderOffset,
            Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
            kInvertRightSide);

    public static final DrivetrainSubsystem DriveTrain = new DrivetrainSubsystem(DrivetrainConstants,
            RobotContainer.m_getFieldCentric, FrontLeft,
            FrontRight, BackLeft, BackRight);
}
