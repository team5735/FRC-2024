package frc.robot.commands.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightPoseEstimatorCommand extends Command {
    LimelightSubsystem limelight;
    DrivetrainSubsystem drivetrain;

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

    LimelightMeasurement lastLimelightPose;

    public static final int AVERAGING_WINDOW = 20;
    LimelightMeasurement averagingMeasurements[] = new LimelightMeasurement[AVERAGING_WINDOW];
    private int index = 0;

    public static final double NEGATION_DETECTION_ERROR = 0.1;

    public LimelightPoseEstimatorCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        LimelightMeasurement measurement = new LimelightMeasurement();
        if (measurement == lastLimelightPose) {
            return;
        }
        if (Math.abs(measurement.normal() - (-lastLimelightPose.normal())) > NEGATION_DETECTION_ERROR) {
            return;
        }
        lastLimelightPose = measurement;

        register(measurement);
        this.drivetrain.addVisionMeasurement(measurement.toPose2d(), measurement.getTimestamp());
    }

    private void register(LimelightMeasurement measurement) {
        averagingMeasurements[index] = measurement;
        index = (index + 1) % AVERAGING_WINDOW;

        LimelightMeasurement average = new LimelightMeasurement();
        for (int i = 0; i < AVERAGING_WINDOW; i++) {
            average.add(averagingMeasurements[i]);
        }
        average.divide(AVERAGING_WINDOW);
    }
}
