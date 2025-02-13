package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ReefAprilTagPositions;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Branch;
import frc.robot.util.Line;
import frc.robot.util.NTDoubleSection;
import frc.robot.util.TunablePIDController;

public class AlignToReef extends Command {
    DrivetrainSubsystem drivetrain;

    Pose2d alignmentTargetTag;
    Line targetLine;

    TunablePIDController omegaController = new TunablePIDController("AlignToReef_omega", 1, 0, 0);
    TunablePIDController lineController = new TunablePIDController("AlignToReef_line", 1, 0, 0);

    NTDoubleSection doubles = new NTDoubleSection(getName(), "movement to line", "omega", "deltaX", "deltaY",
            "line measurement");

    /**
     * Positions the robot in order to score a coral.
     */
    public AlignToReef(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Supplier<Branch> whichBranch) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        this.alignmentTargetTag = ReefAprilTagPositions
                .getClosestTag(drivetrain.getEstimatedPosition().getTranslation());
        this.targetLine = new Line(alignmentTargetTag, "AlignToReef");

        omegaController.setup(alignmentTargetTag.getRotation().unaryMinus().getRadians());
        lineController.setup(0, .02); // we want to be 'at' the Line.
    }

    @Override
    public void execute() {
        Pose2d estimatedPosition = drivetrain.getEstimatedPosition();

        double omega = omegaController.calculate(estimatedPosition.getRotation().getRadians());

        double measurement = targetLine.getPIDMeasurement(estimatedPosition.getTranslation());
        doubles.set("line measurement", measurement);
        double movementTowardsLine = lineController
                .calculate(measurement);
        doubles.set("movement to line", movementTowardsLine);
        Translation2d vectorTowardsLine = targetLine.getVectorFrom(estimatedPosition.getTranslation())
                .times(movementTowardsLine);

        // drivetrain.drive(vectorTowardsLine, omega);
        // drivetrain.drive(0, 0, omega);
        drivetrain.drive(vectorTowardsLine);

        doubles.set("omega", omega);
        doubles.set("deltaX", vectorTowardsLine.getX());
        doubles.set("deltaY", vectorTowardsLine.getY());
    }

    @Override
    public boolean isFinished() {
        return omegaController.atSetpoint();
    }
}
