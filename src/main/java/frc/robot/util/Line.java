package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Line {
    private double slope;

    private double centerX;
    private double centerY;

    public Line(Pose2d pose) {
        slope = Math.tan(pose.getRotation().getRadians());
        centerX = pose.getTranslation().getX();
        centerY = pose.getTranslation().getY();
    }

    /**
     * Modifies the current line, moving it in a direction (todo: which?) by d
     * units.
     */
    public Line offsetBy(double d) {
        Translation2d centerTranslation = new Translation2d(d, Rotation2d.fromRadians(Math.atan(slope) + Math.PI / 2));
        this.centerX += centerTranslation.getX();
        this.centerY += centerTranslation.getY();
        return this;
    }

    /**
     * Returns the distance from position to the line represented by this object.
     *
     * <p>
     * This formula is most similar to this one:
     * https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_formula
     */
    public double getDistance(Translation2d position) {
        return Math.abs(slope * position.getX() - position.getY() + centerY - slope * centerX)
                / Math.sqrt(1 + slope * slope);
    }

    /**
     * {@returns a {@link Translation2d} such that adding it to position returns a
     * point on the line represented by this object}
     */
    public Translation2d getVector(Translation2d position) {
        double perpendicularAngle = Math.atan(slope) + Math.PI / 2;
        return new Translation2d(getDistance(position), Rotation2d.fromRadians(perpendicularAngle));
    }
}
