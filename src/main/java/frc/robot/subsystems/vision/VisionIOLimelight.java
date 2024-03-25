package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // The format for this is [X, Y, Z, roll, pitch, yaw, total latency, tag count,
    // tag span, avg tag distance, average tag area]
    private final DoubleArraySubscriber botpose = table.getDoubleArrayTopic("botpose").subscribe(new double[7]);

    @Override
    public void updateResults(VisionResults results) {
        results.robotPose = getBotPose();
        // Trusting values from team 190
        results.timeStamp = botpose.getLastChange() * 0.000001 - botpose.get()[6] * 0.001;
    }

    private Pose3d getBotPose() {
        double[] limelightPose = botpose.get();
        return new Pose3d(limelightPose[0],
                limelightPose[1],
                limelightPose[2],
                new Rotation3d(limelightPose[3],
                        limelightPose[4],
                        limelightPose[5]));
    }
}
