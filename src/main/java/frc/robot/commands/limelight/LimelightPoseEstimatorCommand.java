package frc.robot.commands.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.NTDoubleSection;

public class LimelightPoseEstimatorCommand extends Command {
    private NTDoubleSection doubles = new NTDoubleSection("limelight_pose_estimator", "estimated X", "estimated Y",
            "estimated Z", "reported X", "reported Y", "reported Z");

    private NTDoubleSection doubles_drivetrain = new NTDoubleSection("drivetrain_pose_estimator", "estimated X",
            "estimated Y", "estimated Z");

    private DrivetrainSubsystem drivetrain;

    class LimelightMeasurement {
        private double x;
        private double y;
        private double z;
        private double pitch;
        private double yaw;
        private double roll;
        private double timestamp;

        public LimelightMeasurement() {
            double[] pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
                    .getDoubleArray(new double[11]);
            x = pose[0];
            y = pose[1];
            z = pose[2];
            pitch = pose[3];
            yaw = pose[4];
            roll = pose[5];
            timestamp = Timer.getFPGATimestamp() - pose[6] / 1000.0;
        }

        public void add(LimelightMeasurement other) {
            x += other.x;
            y += other.y;
            z += other.z;
            pitch += other.pitch;
            yaw += other.yaw;
            roll += other.roll;
        }

        public void divide(double dividend) {
            x /= dividend;
            y /= dividend;
            z /= dividend;
            pitch /= dividend;
            yaw /= dividend;
            roll /= dividend;
        }

        public double normal() {
            double distHorizontal = Math.sqrt(x * x + y * y);
            return Math.sqrt(distHorizontal * distHorizontal + z * z);
        }

        public Pose2d toPose2d() {
            return new Pose2d(x, y, new Rotation2d(yaw));
        }

        public double getTimestamp() {
            return timestamp;
        }
    }

    LimelightMeasurement lastMeasuredPose;

    LimelightMeasurement averagingMeasurements[] = new LimelightMeasurement[LimelightConstants.AVERAGING_WINDOW];
    private int index = 0;

    public LimelightPoseEstimatorCommand(DrivetrainSubsystem drivetrain, LimelightSubsystem limelight) {
        this.drivetrain = drivetrain;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        System.out.println("please, does this work?");
    }

    private double distanceBetween(double a, double b) {
        return Math.abs(a - b);
    }

    @Override
    public void execute() {
        LimelightMeasurement measurement = new LimelightMeasurement();
        if (measurement == lastMeasuredPose) {
            // no new measurement
            return;
        }
        if (distanceBetween(measurement.normal(),
                -lastMeasuredPose.normal()) > LimelightConstants.PERFECT_NEGATION_WINDOW) {
            return;
        }
        lastMeasuredPose = measurement;

        var average = register(measurement);
        report(measurement, average);
        this.drivetrain.addVisionMeasurement(measurement.toPose2d(), measurement.getTimestamp());
    }

    private LimelightMeasurement register(LimelightMeasurement measurement) {
        averagingMeasurements[index] = measurement;
        index = (index + 1) % LimelightConstants.AVERAGING_WINDOW;

        LimelightMeasurement average = new LimelightMeasurement();
        for (int i = 0; i < LimelightConstants.AVERAGING_WINDOW; i++) {
            average.add(averagingMeasurements[i]);
        }
        average.divide(LimelightConstants.AVERAGING_WINDOW);
        return average;
    }

    private void report(LimelightMeasurement measurement, LimelightMeasurement average) {
        doubles.set("estimated X", average.x);
        doubles.set("estimated Y", average.y);
        doubles.set("estimated Z", average.z);
        doubles.set("reported X", measurement.x);
        doubles.set("reported Y", measurement.y);
        doubles.set("reported Z", measurement.z);

        Pose2d pose = this.drivetrain.getState().Pose;
        doubles_drivetrain.set("estimated X", pose.getX());
        doubles_drivetrain.set("estimated Y", pose.getY());
        // there is no estimated Z, because they thought we wouldn't need it. :)
        Rotation3d rot = this.drivetrain.getRotation3d();
        // TODO: double-check that X, Y, and Z are actually roll, pitch, and yaw
        // respectively
        doubles_drivetrain.set("estimated roll", rot.getX());
        doubles_drivetrain.set("estimated pitch", rot.getY());
        doubles_drivetrain.set("estimated yaw", rot.getZ());
    }
}
