package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AprilTagPositions;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.TunablePIDController;

public class AlignToReef extends Command {
    DrivetrainSubsystem drivetrain;

    Pose2d alignmentTargetTag;

    TunablePIDController omegaController;

    /**
     * Positions the robot in order to score a coral.
     */
    public AlignToReef(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.alignmentTargetTag = AprilTagPositions.getClosestTag(drivetrain.getEstimatedPosition().getTranslation());
        omegaController.initialize(alignmentTargetTag.getRotation().getRadians());
    }

    @Override
    public void execute() {
        double omega = omegaController.execute(drivetrain.getEstimatedPosition().getRotation().getRadians());
    }

    @Override
    public boolean isFinished() {
        return omegaController.atSetpoint();
    }
}
