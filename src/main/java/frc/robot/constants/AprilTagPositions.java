package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagPositions {
    // Tags around the Reef, ids 6-11. Field-space, centered at field origin (unlike
    // rest of code .-.)
    public static final Pose2d tags[] = new Pose2d[] {
            // 6
            new Pose2d(
                    new Translation2d(4.700446, -0.719482),
                    new Rotation2d(-60)),
            // 7
            new Pose2d(
                    new Translation2d(5.116498, 0),
                    new Rotation2d(0)),
            // 8
            new Pose2d(
                    new Translation2d(4.700446, 0.719482),
                    new Rotation2d(60)),
            // 9
            new Pose2d(
                    new Translation2d(3.869358, 0.719482),
                    new Rotation2d(120)),
            // 10
            new Pose2d(
                    new Translation2d(3.453306, 0),
                    new Rotation2d(180)),
            // 11
            new Pose2d(
                    new Translation2d(3.869358, -0.719482),
                    new Rotation2d(-120)),
    };
}
