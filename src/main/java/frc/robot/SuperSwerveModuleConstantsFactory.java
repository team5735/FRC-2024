package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

// This file is not actually generated. But I have a problem and I will fix it the wrong way.
public class SuperSwerveModuleConstantsFactory extends SwerveModuleConstantsFactory {
    public SwerveModuleConstants createModuleConstants(
            int steerId,
            int driveId,
            int cancoderId,
            Slot0Configs drivePid,
            Slot0Configs turnPid,
            double cancoderOffset,
            double locationX,
            double locationY,
            boolean driveMotorReversed) {
        return new SwerveModuleConstants()
                .withSteerMotorId(steerId)
                .withDriveMotorId(driveId)
                .withCANcoderId(cancoderId)
                .withCANcoderOffset(cancoderOffset)
                .withLocationX(locationX)
                .withLocationY(locationY)
                .withDriveMotorGearRatio(DriveMotorGearRatio)
                .withSteerMotorGearRatio(SteerMotorGearRatio)
                .withCouplingGearRatio(CouplingGearRatio)
                .withWheelRadius(WheelRadius)
                .withSlipCurrent(SlipCurrent)
                .withSteerMotorGains(turnPid)
                .withDriveMotorGains(drivePid)
                .withSteerMotorClosedLoopOutput(SteerMotorClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(DriveMotorClosedLoopOutput)
                .withSteerMotorInverted(SteerMotorInverted)
                .withDriveMotorInverted(driveMotorReversed)
                .withSpeedAt12VoltsMps(SpeedAt12VoltsMps)
                .withSteerInertia(SteerInertia)
                .withDriveInertia(DriveInertia)
                .withSteerFrictionVoltage(SteerFrictionVoltage)
                .withDriveFrictionVoltage(DriveFrictionVoltage)
                .withFeedbackSource(FeedbackSource);
    }
}
