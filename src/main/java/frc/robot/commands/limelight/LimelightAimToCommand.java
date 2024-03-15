// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightAimToCommand extends Command {
    DrivetrainSubsystem m_drivetrain;
    PIDController m_pid = new PIDController(
            SmartDashboard.getNumber("llv2_turnP", LimelightConstants.TURN_P),
            SmartDashboard.getNumber("llv2_turnI", LimelightConstants.TURN_I),
            SmartDashboard.getNumber("llv2_turnD", LimelightConstants.TURN_D), LimelightConstants.SPF);

    /** Creates a new LimelightAimToCommand. */
    public LimelightAimToCommand(final DrivetrainSubsystem drivetrain, double setpoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drivetrain = drivetrain;

        addRequirements(m_drivetrain);

        m_pid.setTolerance(DrivetrainConstants.TOLERANCE);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.drive(m_pid.calculate(m_drivetrain.getRotation3d().getZ()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_pid.atSetpoint();
    }
}
