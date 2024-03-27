package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Turn90Command extends Command {
    private DrivetrainSubsystem m_drivetrain;
    private PIDController m_controller = new PIDController(2, .2, .15);
    private double m_startPos;
    private final Supplier<Double> getCurrentAngle = () -> m_drivetrain.getState().Pose.getRotation().getRadians();
    private final double m_distance = Math.PI / 2;

    public Turn90Command(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;

        m_controller.enableContinuousInput(-Math.PI, Math.PI);
        m_controller.setTolerance(Constants.TOLERANCE);

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_startPos = getCurrentAngle.get() % (Math.PI * 2);
    }

    @Override
    public void execute() {
        m_drivetrain.drive(m_controller.calculate(getCurrentAngle.get(), m_startPos + m_distance));
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}
