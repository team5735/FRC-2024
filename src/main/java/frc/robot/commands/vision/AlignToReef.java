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
    private DrivetrainSubsystem drivetrain;

    private Pose2d alignmentTargetTag;
    private Line targetLine;

    private TunablePIDController omegaController = new TunablePIDController("AlignToReef_omega", 1, 1, 0);
    private TunablePIDController lineController = new TunablePIDController("AlignToReef_line", 1, 1, 0);

    private NTDoubleSection doubles = new NTDoubleSection(getName(), "omega", "deltaX", "deltaY");

    /**
     * Positions the robot in order to score a coral.
     */
    public AlignToReef(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Supplier<Branch> whichBranch) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain, vision);
    }

    public Supplier<Line> getLineGetter() {
        return () -> this.targetLine;
    }

    @Override
    public void initialize() {
        this.alignmentTargetTag = ReefAprilTagPositions
                .getClosestTag(drivetrain.getEstimatedPosition().getTranslation());
        this.targetLine = new Line(alignmentTargetTag, "AlignToReef");

        omegaController.setup(alignmentTargetTag.getRotation().unaryMinus().getRadians(), 0.1);
        lineController.setup(0, .02); // we want to be 'at' the Line.
    }

    @Override
    public void execute() {
        Pose2d estimatedPosition = drivetrain.getEstimatedPosition();

        double omega = omegaController.calculate(estimatedPosition.getRotation().getRadians());

        double measurement = targetLine.getPIDMeasurement(estimatedPosition.getTranslation());
        double movementTowardsLine = lineController.calculate(measurement);
        Translation2d vectorTowardsLine = targetLine.getVectorFrom(estimatedPosition.getTranslation())
                .times(-movementTowardsLine);

        drivetrain.drive(vectorTowardsLine, omega);

        doubles.set("omega", omega);
        doubles.set("deltaX", vectorTowardsLine.getX());
        doubles.set("deltaY", vectorTowardsLine.getY());
    }

    @Override
    public boolean isFinished() {
        return omegaController.atSetpoint() && lineController.atSetpoint();
    }
}
