package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

public class ShooterHoldNStopCommand extends Command {
    ShooterTopSubsystem m_subsystemTop;
    ShooterBottomSubsystem m_subsystemBottom;

    public ShooterHoldNStopCommand(ShooterTopSubsystem st, ShooterBottomSubsystem sb) {
        m_subsystemTop = st;
        m_subsystemBottom = sb;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystemTop.stop();
        m_subsystemBottom.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
