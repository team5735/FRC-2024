// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.ArrayList;

// Attempts to aim correctly at the field part that the robot can currently see.
public class AimCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final LimelightSubsystem m_subsystem;

    public AimCommand(LimelightSubsystem LLsubsystem) {
        m_subsystem = LLsubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(LLsubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose3d currentBotPose = m_subsystem.getGuessedPosition();
        LimelightTarget_Fiducial[] targets = m_subsystem.getTargetedFiducials();
        // do math n stuff
        SmartDashboard.putNumber("Current bot pose (estimated) x",
                currentBotPose.getX());
        SmartDashboard.putNumber("Current bot pose (estimated) y",
                currentBotPose.getY());
        SmartDashboard.putNumber("Current bot pose (estimated) z",
                currentBotPose.getZ());

        Rotation3d currentBotRot = currentBotPose.getRotation();
        SmartDashboard.putNumber("Current bot rotation (estimated) x",
                currentBotRot.getX());
        SmartDashboard.putNumber("Current bot rotation (estimated) y",
                currentBotRot.getY());
        SmartDashboard.putNumber("Current bot rotation (estimated) z",
                currentBotRot.getZ());

        // is there any better way to do this? I kinda hate this lol but no two
        // ways around it.
        ArrayList<String> targetsAsStrings = new ArrayList<>();
        for (LimelightTarget_Fiducial fid : targets) {
            targetsAsStrings.add("(" + fid.tx + ", " + fid.ty + ", " + fid.ta +
                    ", " + fid.fiducialID + ")");
        }
        String[] strings = new String[targetsAsStrings.size()];
        targetsAsStrings.toArray(strings);
        SmartDashboard.putStringArray(
                "Currently tracked targets (tx, ty, ta, id)", strings);
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
