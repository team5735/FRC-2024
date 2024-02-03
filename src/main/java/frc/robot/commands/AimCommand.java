// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.LimelightSubsystem;

// Attempts to aim correctly at the field part that the robot can currently see.
public class AimCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final LimelightSubsystem m_limelightSubsystem;

    public AimCommand(LimelightSubsystem LLsubsystem) {
        m_limelightSubsystem = LLsubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(LLsubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // attempt to determine the position of the bottom of the hood, roughly
        // (in botspace) for that we need the fiducial's position in botspace
        // distance from the center of the fiducial to the bottom of the hood is
        // 1' 8 7/8"
        LimelightTarget_Fiducial mainTargetedFiducial = m_limelightSubsystem.getMainTargetedFiducial();
        if (mainTargetedFiducial.fiducialID != Constants.kHoodAprilTagFiducialID) {
            return;
        }

        // x = forward, y = right, z = up
        Translation3d robotToFiducial = mainTargetedFiducial.getTargetPose_RobotSpace().getTranslation();
        // TODO: determine units that limelight gives
        Translation3d robotToHood = robotToFiducial.plus(
                new Translation3d(0, Constants.kHoodAprilTagHeight, 0));
        Translation3d robotShooterOrigin = new Translation3d(
                -Constants.kIntakeDistBehind, 0, Constants.kIntakeHeight);
        Translation3d shooterToHood = robotToHood.minus(robotShooterOrigin);

        // could also use Math.atan2
        // norm is the distance from the origin
        double requiredShooterAngle = Math.atan(shooterToHood.getZ() /
                shooterToHood.toTranslation2d().getNorm());

        SmartDashboard.putNumber("requiredShooterAngle", requiredShooterAngle);
        SmartDashboard.putNumber("distanceToTarget", shooterToHood.getX());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // just does logging for now. in the future, we may have it turn;
        // that'd take more commands.
        return true;
    }
}
