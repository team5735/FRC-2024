package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BrakeCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;

    public BrakeCommand(DrivetrainSubsystem train) {
        addRequirements(train);
        m_drivetrain = train;
    }

    public void execute() {
        m_drivetrain.brake();
    }
}
