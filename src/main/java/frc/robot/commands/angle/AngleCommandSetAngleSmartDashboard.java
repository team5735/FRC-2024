package frc.robot.commands.angle;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AngleConstants;
import frc.robot.subsystems.AngleSubsystem;

public class AngleCommandSetAngleSmartDashboard extends Command {
    private AngleSubsystem m_subsystem;
    private double m_setpoint;

    public AngleCommandSetAngleSmartDashboard(AngleSubsystem s) {
        m_subsystem = s;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("lmfao");
        m_setpoint = SmartDashboard.getNumber("testShootAngle", AngleConstants.ANGLE_START_POS_DEG);
        m_subsystem.setSetpoint(m_setpoint);
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
        return m_subsystem.isAtPosition(m_setpoint);
    }
}
