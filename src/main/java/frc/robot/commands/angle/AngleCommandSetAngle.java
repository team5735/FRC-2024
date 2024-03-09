package frc.robot.commands.angle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleSubsystem;

public class AngleCommandSetAngle extends Command{
    AngleSubsystem m_subsystem;

    public AngleCommandSetAngle(AngleSubsystem s) {
        m_subsystem = s;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.setSetpoint(SmartDashboard.getNumber("angleNewSetpoint", 90));
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
