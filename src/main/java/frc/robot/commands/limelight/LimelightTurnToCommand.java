// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.NTBooleanSection;
import frc.robot.util.NTDoubleSection;
import frc.robot.util.TunableNumber;

/**
 * Uses a {@link PIDController} to turn the drivetrain to a specified angle.
 * Accounts for the pigeon's offset.
 */
public class LimelightTurnToCommand extends Command {
    DrivetrainSubsystem m_drivetrain;
    LimelightSubsystem m_limelight;
    PIDController m_pid;
    double m_pigeonStartingNumber;

    private final NTDoubleSection m_doubles = new NTDoubleSection("limelight", "drivetrain omega", "measurement",
            "setpoint");
    private final NTBooleanSection m_booleans = new NTBooleanSection("limelight", "aiming");

    private final TunableNumber m_kP = new TunableNumber("limelight", "kP", LimelightConstants.TURN_P);
    private final TunableNumber m_kI = new TunableNumber("limelight", "kI", LimelightConstants.TURN_I);
    private final TunableNumber m_kD = new TunableNumber("limelight", "kD", LimelightConstants.TURN_D);

    /**
     * Creates a new LimelightTurnToCommand. Initializes everything and logs the
     * setpoint.
     */
    public LimelightTurnToCommand(final DrivetrainSubsystem drivetrain, final LimelightSubsystem limelight,
            final double offset) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;

        addRequirements(m_drivetrain);

        m_pid = new PIDController(m_kP.get(), m_kI.get(), m_kD.get());

        m_pid.setTolerance(Constants.TOLERANCE);
        m_pid.setSetpoint(LimelightAimCommand.positiveToPosNeg(m_drivetrain.getRotation3d().getZ() + offset));
        m_pid.enableContinuousInput(-Math.PI, Math.PI);

        m_pigeonStartingNumber = m_drivetrain.getRotation3d().getZ();

        m_doubles.set("setpiont", m_pid.getSetpoint());
    }

    @Override
    public void initialize() {
    }

    /**
     * Uses the output of the PID to drive the drivetrain towards the setpoint.
     */
    @Override
    public void execute() {
        double measurement = getMeasurement();
        m_doubles.set("measurement", measurement);
        double omega = m_pid.calculate(measurement);
        m_doubles.set("drivetrain omega", omega);
        m_drivetrain.drive(omega);
    }

    /**
     * Stops the drivetrain and sets aiming to false so it's known that aiming is
     * done.
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0);
        m_booleans.set("aiming", false);
    }

    /**
     * Returns the drivetrain's current rotation as reported by the Pigeon2.
     */
    private double getMeasurement() {
        return m_drivetrain.getRotation3d().getZ();
    }

    /**
     * Determines whether the absolute difference between the setpoint and the
     * measurement is less than the tolerance.
     */
    @Override
    public boolean isFinished() {
        return Math.abs(getMeasurement() - m_pid.getSetpoint()) < Constants.TOLERANCE;
    }
}
