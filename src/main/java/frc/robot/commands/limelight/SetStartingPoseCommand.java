package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SetStartingPoseCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    private final LimelightSubsystem m_limelight;
    private boolean m_done;

    public SetStartingPoseCommand(DrivetrainSubsystem drivetrain, LimelightSubsystem limelighte) {
        m_drivetrain = drivetrain;
        m_limelight = limelighte;
    }

    @Override
    public void initialize() {
        m_drivetrain.seedFieldRelative(m_limelight.getBotPose2d());
        m_done = true;
    }

    @Override
    public boolean isFinished() {
        return m_done;
    }
}
