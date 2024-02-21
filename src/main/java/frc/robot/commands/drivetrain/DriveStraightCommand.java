package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveStraightCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    public DriveStraightCommand(DrivetrainSubsystem d) {
        m_drivetrain = d;
        addRequirements(m_drivetrain);
    }

    public void execute() {
        m_drivetrain.driveClosedLoop(4.00, 0, 0);
    }
}
