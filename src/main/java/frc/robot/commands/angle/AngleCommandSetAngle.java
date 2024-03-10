package frc.robot.commands.angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleSubsystem;

public class AngleCommandSetAngle extends Command {
    AngleSubsystem m_subsystem;
    double m_setpoint;

    public AngleCommandSetAngle(AngleSubsystem s, double angle) {
        m_subsystem = s;
        m_setpoint = angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.setSetpoint(m_setpoint);
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
        return m_subsystem.isAtSetpoint();
    }

}
