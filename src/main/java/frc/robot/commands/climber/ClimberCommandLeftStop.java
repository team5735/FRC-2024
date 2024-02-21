package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommandLeftStop extends Command {
    ClimberSubsystem m_subsystem;

    public ClimberCommandLeftStop(ClimberSubsystem s) {
        addRequirements(s);
        m_subsystem = s;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.stopLeft();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println("hellotest");
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
