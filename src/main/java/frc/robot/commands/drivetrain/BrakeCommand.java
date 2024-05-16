package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A simple {@link Command} intended to brake a {@link DrivetrainSubsystem} by
 * calling its {@code brake()} method repeatedly.
 */
public class BrakeCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;

    /**
     * Initializes a new BrakeCommand, by requiring and storing the value of the
     * passed-in {@link DrivetrainSubsystem}.
     * 
     * @param train - the {@link DrivetrainSubsystem} to brake and require
     */
    public BrakeCommand(DrivetrainSubsystem train) {
        addRequirements(train);
        m_drivetrain = train;
    }

    /**
     * Repeatedly brakes the passed-in {@link DrivetrainSubsystem} by calling its {@code brake()} method.
     */
    public void execute() {
        m_drivetrain.brake();
    }
}
