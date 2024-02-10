// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightConstants;
import frc.robot.libraries.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.LimelightSubsystem;

/** An example command that uses an example subsystem. */
public class AimCommandV2 extends Command {
    private LimelightSubsystem m_subsystem;
    private boolean m_targetAcquired = false;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AimCommandV2(final LimelightSubsystem subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        m_subsystem = subsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        LimelightTarget_Fiducial[] targets = m_subsystem.getTargetedFiducials();
        if (targets.length == 0) {
            return;
        }
        m_targetAcquired = true;

        // coordinate system: x along long side (facing TODO team), y along short side
        // (facing TODO), z up
        Pose3d currentRobotPos = targets[0].getRobotPose_FieldSpace();
        Translation3d centerOfHood = LimelightConstants.HOOD_POS_TRANSLATION;
        Translation3d robotToHood = centerOfHood.minus(currentRobotPos.getTranslation());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_targetAcquired;
    }
}
