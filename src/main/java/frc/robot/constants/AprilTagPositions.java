package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagPositions {
    /**
     * Tags around the blue alliance reef. Field-space, centered at field origin
     * (unlike the rest of the robot code .-.)
     *
     * <p>
     * Here's a diagram of the reefs with their tags.
     * The 'x' in the middle is the field origin. The arrow is θ=0
     * The 'y' in the bottom left is the blue alliance field origin.
     * In reality, each reef is a hexagon (so no __ sides), but I couldn't make that
     * look nice with ASCII.
     * <code>
     * Blue alliance             Red alliance
     * v                                    v
     * / ***************************************** \
     * |         __                     __         |
     * |      19/  \20                9/  \8       |
     * |     18|    |21      x →    10|    |7      |
     * |      17\__/22               11\__/6       |
     * |                                           |
     * \y***************************************** /
     * </code>
     */
    private static final Pose2d TAGS[] = new Pose2d[] {
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
