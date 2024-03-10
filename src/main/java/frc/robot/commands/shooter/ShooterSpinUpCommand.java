package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

public class ShooterSpinUpCommand extends Command {
    ShooterTopSubsystem m_subsystemTop;
    ShooterBottomSubsystem m_subsystemBottom;

    public ShooterSpinUpCommand(ShooterTopSubsystem st, ShooterBottomSubsystem sb) {
        m_subsystemTop = st;
        m_subsystemBottom = sb;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystemTop.start();
        m_subsystemBottom.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_subsystemTop.stop();
            m_subsystemBottom.stop();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_subsystemTop.isSpunUp() && m_subsystemBottom.isSpunUp();
    }
}
