// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.NTBooleanSection;
import frc.robot.util.NTDoubleSection;
import frc.robot.util.TunableNumber;

/**
 * Uses a {@link PIDController} to turn the drivetrain to a specified angle.
 * Accounts for the pigeon's offset.
 */
public class LimelightTurnToCommand extends Command {
    DrivetrainSubsystem m_drivetrain;
    PIDController m_pid;
    double m_pigeonStartingNumber;
    Supplier<Double> setpointGetter;
    private Supplier<Double> getMeasurement;

    private final NTDoubleSection m_doubles = new NTDoubleSection("limelight turn", "drivetrain omega", "measurement",
            "setpoint", "position error");
    private final NTBooleanSection m_booleans = new NTBooleanSection("limelight turn", "aiming");

    private final TunableNumber m_kP = new TunableNumber("limelight", "kP", LimelightConstants.TURN_P);
    private final TunableNumber m_kI = new TunableNumber("limelight", "kI", LimelightConstants.TURN_I);
    private final TunableNumber m_kD = new TunableNumber("limelight", "kD", LimelightConstants.TURN_D);

    /** Creates a new LimelightTurnToCommand. */
    public LimelightTurnToCommand(final DrivetrainSubsystem drivetrain,
            Supplier<Double> setpointGetter, Supplier<Double> drivetrainRotationSupplier) {
        m_drivetrain = drivetrain;

        addRequirements(m_drivetrain);

        this.setpointGetter = setpointGetter;
        this.getMeasurement = drivetrainRotationSupplier;
    }

    @Override
    public void initialize() {
        System.out.println("started");
        m_pid = new PIDController(m_kP.get(), m_kI.get(), m_kD.get());

        m_pid.setTolerance(Constants.TOLERANCE);
        // m_pid.setSetpoint(LimelightAimCommand.positiveToPosNeg(m_drivetrain.getRotation3d().getZ()
        // + offset));
        m_pid.setSetpoint(setpointGetter.get());
        m_pid.enableContinuousInput(-Math.PI, Math.PI);

        m_pigeonStartingNumber = m_drivetrain.getRotation3d().getZ();

        m_doubles.set("setpoint", m_pid.getSetpoint());
        m_booleans.set("aiming", true);
    }

    /**
     * Uses the output of the PID to drive the drivetrain towards the setpoint.
     */
    @Override
    public void execute() {
        double measurement = getMeasurement.get();
        double omega = m_pid.calculate(measurement);
        if (Math.abs(omega) > 1) {
            omega = 1 * Math.signum(omega);
        }
        m_doubles.set("drivetrain omega", omega);
        m_drivetrain.drive(omega);
        m_doubles.set("position error", m_pid.getPositionError());
        SmartDashboard.putNumber("drivetrain reported theta", getMeasurement.get());
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
     * Determines whether the absolute difference between the setpoint and the
     * measurement is less than the tolerance.
     */
    @Override
    public boolean isFinished() {
        return Math.abs(getMeasurement.get() - m_pid.getSetpoint()) < Constants.TOLERANCE;
    }
}
