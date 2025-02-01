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
public class VisionTransRot extends Command {
    DrivetrainSubsystem m_drivetrain;
    PIDController pid;
    double m_pigeonStartingNumber;
    Supplier<Double> rotationSetpointGetter;

    private final NTDoubleSection m_doubles = new NTDoubleSection("limelight turn", "drivetrain omega", "measurement",
            "setpoint", "position error");
    private final NTBooleanSection m_booleans = new NTBooleanSection("limelight turn", "aiming");

    private final TunableNumber kP = new TunableNumber("limelight", "kP", LimelightConstants.TURN_P);
    private final TunableNumber kI = new TunableNumber("limelight", "kI", LimelightConstants.TURN_I);
    private final TunableNumber kD = new TunableNumber("limelight", "kD", LimelightConstants.TURN_D);

    public VisionTransRot(final DrivetrainSubsystem drivetrain,
            Supplier<Double> rotationSetpoint) {
        m_drivetrain = drivetrain;

        addRequirements(m_drivetrain);

        this.rotationSetpointGetter = rotationSetpoint;
    }

    @Override
    public void initialize() {
        System.out.println("started");
        pid = new PIDController(kP.get(), kI.get(), kD.get());

        pid.setTolerance(Constants.TOLERANCE);
        pid.setSetpoint(rotationSetpointGetter.get());
        pid.enableContinuousInput(-Math.PI, Math.PI);

        m_pigeonStartingNumber = m_drivetrain.getRotation3d().getZ();

        m_doubles.set("setpoint", pid.getSetpoint());
        m_booleans.set("aiming", true);
    }

    /**
     * Uses the output of the PID to drive the drivetrain towards the setpoint.
     */
    @Override
    public void execute() {
        double measurement = m_drivetrain.getEstimatedPosition().getRotation().getRadians();
        double omega = pid.calculate(measurement);
        if (Math.abs(omega) > 1) {
            omega = 1 * Math.signum(omega);
        }
        m_doubles.set("drivetrain omega", omega);
        m_drivetrain.drive(omega);
        m_doubles.set("position error", pid.getPositionError());
        SmartDashboard.putNumber("drivetrain reported theta",
                m_drivetrain.getEstimatedPosition().getRotation().getRadians());
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
        return Math.abs(m_drivetrain.getEstimatedPosition().getRotation().getRadians()
                - pid.getSetpoint()) < Constants.TOLERANCE;
    }
}
