package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.libraries.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SetStartingPoseCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    private final LimelightSubsystem m_lime;
    private boolean m_done;
    public SetStartingPoseCommand(DrivetrainSubsystem sub, LimelightSubsystem l) {
        m_drivetrain = sub;
        m_lime = l;
    }

    public void initialize() {
        m_drivetrain.setPose(m_lime.getBotPose2d());
        m_done = true;
    }

    @Override
    public boolean isFinished() {
        return m_done;
    }
}
