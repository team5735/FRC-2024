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
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightAimToCommand extends Command {
    DrivetrainSubsystem m_drivetrain;
    LimelightSubsystem m_limelight;
    PIDController m_pid = new PIDController(
            SmartDashboard.getNumber("llv2_turnP", LimelightConstants.TURN_P),
            SmartDashboard.getNumber("llv2_turnI", LimelightConstants.TURN_I),
            SmartDashboard.getNumber("llv2_turnD", LimelightConstants.TURN_D));

    /** Creates a new LimelightAimToCommand. */
    public LimelightAimToCommand(final DrivetrainSubsystem drivetrain, final LimelightSubsystem limelight,
            final double setpoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drivetrain = drivetrain;
        m_limelight = limelight;

        addRequirements(m_drivetrain);

        m_pid.setTolerance(DrivetrainConstants.TOLERANCE);
        m_pid.setSetpoint(setpoint);
        m_pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("llTurnTo_measurement", getMeasurement());
        double omega = m_pid.calculate(getMeasurement());
        m_drivetrain.drive(omega);
        SmartDashboard.putNumber("llTurnTo_omega", omega);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0);
        SmartDashboard.putBoolean("llv2_aiming", false);
    }

    private double getMeasurement() {
        return m_drivetrain.getRotation3d().getZ();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(getMeasurement() - m_pid.getSetpoint()) < DrivetrainConstants.TOLERANCE;
    }
}
