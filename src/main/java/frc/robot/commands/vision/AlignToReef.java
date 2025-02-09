package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ReefAprilTagPositions;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Branch;
import frc.robot.util.Line;
import frc.robot.util.NTDoubleSection;
import frc.robot.util.TunablePIDController;

public class AlignToReef extends Command {
    DrivetrainSubsystem drivetrain;

    Pose2d alignmentTargetTag;
    Line targetLine;

    TunablePIDController omegaController = new TunablePIDController("AlignToReef_omega");
    TunablePIDController lineController = new TunablePIDController("AlignToReef_line");

    NTDoubleSection doubles = new NTDoubleSection(getName(), "movement to line", "omega", "deltaX", "deltaY");

    /**
     * Positions the robot in order to score a coral.
     */
    public AlignToReef(DrivetrainSubsystem drivetrain, Supplier<Branch> whichBranch) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.alignmentTargetTag = ReefAprilTagPositions
                .getClosestTag(drivetrain.getEstimatedPosition().getTranslation());
        this.targetLine = new Line(alignmentTargetTag, "AlignToReef");

        omegaController.setup(alignmentTargetTag.getRotation().unaryMinus().getRadians());
        lineController.setup(0, .1); // we want to be 'at' the Line.
    }

    @Override
    public void execute() {
        Pose2d estimatedPosition = drivetrain.getEstimatedPosition();

        double omega = omegaController.calculate(estimatedPosition.getRotation().getRadians());

        double movementTowardsLine = lineController
                .calculate(targetLine.getPIDMeasurement(estimatedPosition.getTranslation()));
        doubles.set("movement to line", movementTowardsLine);
        Translation2d vectorTowardsLine = targetLine.getVectorFrom(estimatedPosition.getTranslation())
                .times(movementTowardsLine);

        // drivetrain.drive(vectorTowardsLine, omega);
        drivetrain.drive(0, 0, omega);

        doubles.set("omega", omega);
        doubles.set("deltaX", vectorTowardsLine.getX());
        doubles.set("deltaY", vectorTowardsLine.getY());
    }

    @Override
    public boolean isFinished() {
        return omegaController.atSetpoint() && lineController.atSetpoint();
    }
}
