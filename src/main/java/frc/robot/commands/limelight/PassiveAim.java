package frc.robot.commands.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * This Command attempts to aim to angle changer at the speaker while still
 * being able to move the drivetrain. It passes vision measurements to the
 * drivetrain to do this.
 */
public class PassiveAim extends Command {
    private AngleSubsystem angleChanger;
    private LimelightSubsystem limelight;
    private DrivetrainSubsystem drivetrain;

    private Pose2d lastReportedPoseFromLimelight;

    /**
     * Creates a new PassiveAim command. This requires the angle changer and the
     * limelight but not the drivetrain because it doesn't move the drivetrain and
     * additionally doesn't want to interfere with its default command
     * ({@link DriveCommand}).
     */
    public PassiveAim(AngleSubsystem angleChanger, LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
        this.angleChanger = angleChanger;
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        addRequirements(angleChanger, limelight);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d latestRobotPoseLimelight = this.limelight.getBotPose().toPose2d();
        if (latestRobotPoseLimelight != this.lastReportedPoseFromLimelight) {
            this.lastReportedPoseFromLimelight = latestRobotPoseLimelight;
            this.drivetrain.addVisionMeasurement(latestRobotPoseLimelight, Timer.getFPGATimestamp());
        }
    }
}
