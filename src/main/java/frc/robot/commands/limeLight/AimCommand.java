// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightConstants;
import frc.robot.libraries.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.LimelightSubsystem;

// Attempts to aim correctly at the field part that the robot can currently see.
public class AimCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final LimelightSubsystem m_limelightSubsystem;
    private boolean m_aimed = false;

    public AimCommand(LimelightSubsystem subsystem) {
        m_limelightSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // attempt to determine the position of the bottom of the hood, roughly
        // (in botspace) for that we need the fiducial's position in botspace
        // distance from the center of the fiducial to the bottom of the hood is
        // 1' 8 7/8"
        LimelightTarget_Fiducial[] fiducials = m_limelightSubsystem.getTargetedFiducials();
        if (fiducials.length == 0) {
            return;
        }
        LimelightTarget_Fiducial mainTargetedFiducial = fiducials[0];
        if (mainTargetedFiducial.fiducialID != LimelightConstants.SPEAKER_LEFT_TAG_FIDUCIALID) {
            return;
        }

        // robot space: +x is right, +y is DOWN, +z is forward
        Translation3d robotToFiducial = mainTargetedFiducial.getTargetPose_RobotSpace().getTranslation();
        // limelight units are meters
        Translation3d robotToHood = robotToFiducial.minus(
                new Translation3d(0, LimelightConstants.SPEAKER_LEFT_TAG_HEIGHT, 0));
        Translation3d robotShooterOrigin = new Translation3d(
                -LimelightConstants.PIVOT_OFFSET_X, 0, LimelightConstants.PIVOT_OFFSET_Z);
        Translation3d shooterToHood = robotToHood.minus(robotShooterOrigin);

        // could also use Math.atan2
        // norm is the distance from the origin
        double requiredShooterAngle = Math.atan(-shooterToHood.getY() /
                new Translation2d(shooterToHood.getX(), shooterToHood.getZ()).getNorm());
        double requiredShooterAngleDeg = Math.toDegrees(requiredShooterAngle);

        SmartDashboard.putNumber("requiredShooterAngle", requiredShooterAngleDeg);
        SmartDashboard.putNumber("distanceToTarget", shooterToHood.getZ());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_aimed;
    }
}
