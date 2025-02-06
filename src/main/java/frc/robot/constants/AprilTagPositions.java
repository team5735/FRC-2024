package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagPositions {
    // Tags around the Reef, ids 6-11. Field-space, centered at field origin (unlike
    // rest of code .-.)
    Pose2d tags[] = new Pose2d[] {
            // 6
            new Pose2d(
                    new Translation2d(4.700446, -0.719682),
                    new Rotation2d(-60)),
            // 7
            new Pose2d(
                    new Translation2d(5.116498, 0),
                    new Rotation2d(0)),
            // 8
            new Pose2d(
                    new Translation2d(5.116498, 0.719682),
                    new Rotation2d(60)),
    };
}
