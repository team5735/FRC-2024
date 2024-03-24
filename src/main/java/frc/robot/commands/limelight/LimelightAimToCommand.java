// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightAimToCommand extends Command {
    DrivetrainSubsystem m_drivetrain;
    LimelightSubsystem m_limelight;
    PIDController m_pid;
    double m_pigeonStartingNumber;

    private final NetworkTable m_node = NetworkTableInstance.getDefault().getTable("limelight");
    private final BooleanPublisher m_aimingPublisher = m_node.getBooleanTopic("aiming").publish();
    private final DoublePublisher m_drivetrainOmegaPublisher = m_node.getDoubleTopic("drivetrainOmega").publish();
    private final DoublePublisher m_measurementPublisher = m_node.getDoubleTopic("measurement").publish();
    private final DoublePublisher m_setpointPublisher = m_node.getDoubleTopic("setpoint").publish();

    private final DoubleSubscriber m_kP = m_node.getDoubleTopic("kP").subscribe(LimelightConstants.TURN_P);
    private final DoubleSubscriber m_kI = m_node.getDoubleTopic("kI").subscribe(LimelightConstants.TURN_I);
    private final DoubleSubscriber m_kD = m_node.getDoubleTopic("kD").subscribe(LimelightConstants.TURN_D);

    /** Creates a new LimelightAimToCommand. */
    public LimelightAimToCommand(final DrivetrainSubsystem drivetrain, final LimelightSubsystem limelight,
            final double offset) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;

        addRequirements(m_drivetrain);

        m_pid = new PIDController(m_kP.get(), m_kI.get(), m_kD.get());

        m_pid.setTolerance(DrivetrainConstants.TOLERANCE);
        m_pid.setSetpoint(LimelightAimCommand.positiveToPosNeg(m_drivetrain.getRotation3d().getZ() + offset));
        m_pid.enableContinuousInput(-Math.PI, Math.PI);

        m_pigeonStartingNumber = m_drivetrain.getRotation3d().getZ();

        m_setpointPublisher.set(m_pid.getSetpoint());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double measurement = getMeasurement();
        m_measurementPublisher.set(measurement);
        double omega = m_pid.calculate(measurement);
        m_drivetrainOmegaPublisher.set(omega);
        m_drivetrain.drive(omega);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0);
        m_aimingPublisher.set(false);
    }

    private double getMeasurement() {
        return m_drivetrain.getRotation3d().getZ();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(getMeasurement() - m_pid.getSetpoint()) < DrivetrainConstants.TOLERANCE;
        // return m_pid.atSetpoint();
    }
}
