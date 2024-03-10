package frc.robot.commands.angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleSubsystem;

public class AngleCommandPIDReset extends Command {
    AngleSubsystem m_subsystem;

    public AngleCommandPIDReset(AngleSubsystem s) {
        m_subsystem = s;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.pidReset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
