package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
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

    private TunablePIDController omegaController = new TunablePIDController("AlignToReef_omega", 1, 0, 0);
    private TunablePIDController lineController = new TunablePIDController("AlignToReef_line", 1, 0, 0);

    private NTDoubleSection doubles = new NTDoubleSection(getName(), "omega", "deltaX", "deltaY");

    private Supplier<Double> input;

    private static boolean infinite = true;

    /**
     * Positions the robot in order to score a coral.
     */
    public AlignToReef(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Supplier<Branch> whichBranch,
            Supplier<Double> input) {
        this.drivetrain = drivetrain;
        this.input = input;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        this.alignmentTargetTag = ReefAprilTagPositions
                .getClosestTag(drivetrain.getEstimatedPosition().getTranslation());
        this.targetLine = new Line(alignmentTargetTag, "AlignToReef");

        omegaController.setup(alignmentTargetTag.getRotation().unaryMinus().getRadians(), 0.05);
        lineController.setup(0, .02); // we want to be 'at' the Line.

        SmartDashboard.putBoolean("aligning", true);
    }

    @Override
    public void execute() {
        Pose2d estimatedPosition = drivetrain.getEstimatedPosition();

        double omega = omegaController.calculate(estimatedPosition.getRotation().getRadians());

        double measurement = targetLine.getPIDMeasurement(estimatedPosition.getTranslation());
        double movementTowardsLine = lineController.calculate(measurement);
        Translation2d drivetrainMovement = targetLine.getVectorFrom(estimatedPosition.getTranslation())
                .times(-movementTowardsLine);

        doubles.set("omega", omega);
        doubles.set("deltaX", drivetrainMovement.getX());
        doubles.set("deltaY", drivetrainMovement.getY());

        Translation2d alongLineNormalized = this.targetLine.getVectorAlongLine();
        Translation2d alongLine = alongLineNormalized
                .times(MathUtil.applyDeadband(this.input.get(), DrivetrainConstants.DEADBAND));
        drivetrainMovement = drivetrainMovement.plus(alongLine);

        drivetrain.drive(drivetrainMovement, omega);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("aligning", false);
    }

    private boolean atSetpoint() {
        return lineController.atSetpoint() && omegaController.atSetpoint();
    }

    @Override
    public boolean isFinished() {
        return !infinite && atSetpoint();
    }
}
