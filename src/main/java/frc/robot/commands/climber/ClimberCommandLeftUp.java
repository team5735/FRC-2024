package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberCommandLeftUp extends Command {
    ClimberSubsystem m_subsystem;

    public ClimberCommandLeftUp(ClimberSubsystem s) {
        addRequirements(s);
        m_subsystem = s;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.leftUp();
        // System.out.println("hellotest");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopLeft();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}