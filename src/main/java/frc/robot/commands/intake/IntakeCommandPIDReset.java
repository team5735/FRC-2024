package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandPIDReset extends Command {
    IntakeSubsystem m_subsystem;

    public IntakeCommandPIDReset(IntakeSubsystem s) {
        addRequirements(s);
        m_subsystem = s;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // PLEASE HELP I AM CONFUSE
        // m_subsystem.resetThat();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
