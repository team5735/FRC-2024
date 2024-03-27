// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.AllianceSwitcher;
import frc.robot.util.NTBooleanSection;
import frc.robot.util.NTDoubleSection;

/**
 * A command that uses data from the {@link LimelightSubsystem} to aim the
 * drivetrain and the angle changer at the speaker. Uses LimelightTurnToCommand
 * to turn the drivetrain and sets the angle changer subsystem's setpoint.
 */
public class LimelightAimCommand extends Command {
    private LimelightSubsystem limelight;
    private DrivetrainSubsystem drivetrain;
    private AngleSubsystem angleChanger;
    private boolean targetAcquired = false;
    private Watchdog watchdog = new Watchdog(0.02, () -> {
    });

    private final NTDoubleSection m_doubles = new NTDoubleSection("limelight", "current rotation", "hood distance",
            "cannot aim distance", "drivetrain speed x", "drivetrain speed y", "desired drivetrain angle",
            "hood vector x", "hood vector y", "angle changer radians");

    private final NTBooleanSection m_booleans = new NTBooleanSection("limelight", "aiming", "spinning");

    private static final Translation3d m_hoodPos = AllianceSwitcher.compute(LimelightConstants.HOOD_POS);

    /**
     * Creates a new LimelightAimCommand. This is responsible for turning the
     * robot to face the hood and for setting the angle changer to the correct angle
     * to shoot a NOTE into the hood.
     *
     * @param limelight  The limelight that is used for aiming
     * @param drivetrain The drivetrain, used to turn automatically and aim
     *                   horizontally
     * @param angle      The angle changer, used to aim vertically
     */
    public LimelightAimCommand(final LimelightSubsystem limelight, final DrivetrainSubsystem drivetrain,
            final AngleSubsystem angleSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(limelight, drivetrain);
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        angleChanger = angleSubsystem;
        targetAcquired = false;
    }

    @Override
    public void initialize() {
    }

    /**
     * Attempts to aim. This is in execute() so that if we don't have enough
     * targets, we can just turn.
     */
    @Override
    public void execute() {
        watchdog.reset();

        Pose3d botPose = limelight.getBotPose();
        watchdog.addEpoch("get bot pose");
        if (limelight.getNumTargets() < 2) {
            m_booleans.set("spinning", true);
            drivetrain.drive(LimelightConstants.CLUELESS_TURN_SPEED);
            return;
        }
        watchdog.addEpoch("check num targets");

        m_booleans.set("aiming", true);
        targetAcquired = true;
        watchdog.addEpoch("a target has been acquired");

        // coordinate system for field-oriented limelight targeting: x along long side
        // with positive towards red alliance, y along short side such that positive is
        // facing the long side that has red on the right and blue on the left, z up,
        // positive theta is counterclockwise and theta 0 is facing the red alliance
        // speaker.

        Pose2d currentRobotPose = botPose.toPose2d();
        Translation3d hoodPos = m_hoodPos;
        checkBotCanAim(currentRobotPose.getTranslation(), hoodPos);
        watchdog.addEpoch("checked bot can aim");

        Translation2d robotToHood = hoodPos.toTranslation2d().minus(currentRobotPose.getTranslation());
        aimHorizontally(robotToHood);
        m_doubles.set("current rotation", currentRobotPose.getRotation().getRadians());
        watchdog.addEpoch("aimed horizontally");

        Translation3d robotPosTranslation3d = new Translation3d(currentRobotPose.getX(), currentRobotPose.getY(), 0);
        double robotRotation = currentRobotPose.getRotation().getRadians();
        Translation3d angleChangerPosition = robotPosTranslation3d.plus(LimelightConstants.ANGLE_CHANGER_OFFSET
                .rotateBy(new Rotation3d(0, 0, robotRotation)));
        aimVertically(angleChangerPosition, hoodPos);
        watchdog.addEpoch("aimed vertically");

        m_doubles.set("hood distance", robotToHood.getNorm());
        watchdog.disable();
        watchdog.printEpochs();
    }

    // checks to see if the robot is within reasonable shooting range of the target
    /**
     * Determines whether the robot is close enough to the speaker to be able to
     * reasonably shoot. The threshold is set high enough that this function
     * practically never does anything, but it's there. It also would try to drive
     * towards the speaker, but if the robot is far enough that it needs to drive,
     * it can't see the AprilTags well enough anyway.
     *
     * @param robotPosition  The current robot's position in field space
     * @param targetPosition The target's position in field space
     */
    private void checkBotCanAim(Translation2d robotPosition, Translation3d targetPosition) {
        boolean botIsCloseEnough = robotPosition
                .getDistance(targetPosition.toTranslation2d()) < LimelightConstants.BOT_SHOOTING_DISTANCE;
        if (botIsCloseEnough) {
            return;
        }
        m_doubles.set("cannot aim distance",
                robotPosition.getDistance(m_hoodPos.toTranslation2d()));
        Translation2d robotToTarget = targetPosition.toTranslation2d().minus(robotPosition);
        Translation2d desiredVelocity = robotToTarget.div(robotToTarget.getNorm()) // normalize the vector
                .times(LimelightConstants.DRIVETRAIN_MOVEMENT_SPEED); // set magnitude to allowed drivetrain
                                                                      // movement speed
        // m_drivetrain.drive(desiredVelocity);
        m_doubles.set("drivetrain speed x", desiredVelocity.getX());
        m_doubles.set("drivetrain speed y", desiredVelocity.getY());
    }

    /**
     * Aims horizontally. This function uses atan2(double, double) to compute the
     * angle at which the drivetrain should face in order to see the speaker. It
     * then passes that angle to a new {@link LimelightTurnToCommand} which does the
     * PID work of turning the drivetrain.
     *
     * @param currentRobotPoseToTarget The vector between the current robot pose and
     *                                 the target.
     */
    private void aimHorizontally(Translation2d currentRobotPoseToTarget) {
        double drivetrainDesiredAngle = Math.atan2(currentRobotPoseToTarget.getY(), currentRobotPoseToTarget.getX());
        double offset = posNegToPositive(drivetrainDesiredAngle);

        m_doubles.set("desired drivetrain angle", drivetrainDesiredAngle);
        m_doubles.set("hood vector x", currentRobotPoseToTarget.getX());
        m_doubles.set("hood vector y", currentRobotPoseToTarget.getY());

        new LimelightTurnToCommand(drivetrain, limelight, offset).schedule();
    }

    /**
     * Determines the angle that the angle changer would have to point at in order
     * to shoot into the target from angler. Angler is the vector of the angle
     * changer's pivot, and target is the vector of the hood of the speaker -- where
     * we're trying to shoot.
     *
     * <p>
     * Because of the mechanical design of the angle changer, we have to perform
     * some more advanced trig than just another atan2. The problem is interpreted
     * and solved as such:
     *
     * <p>
     * Given: the coordinates and radius of circle O and the coordinates of point P,
     * solve for the angle between the radius OA where A is the point where tangent
     * PA meets the circumference of circle O and the horizon, which is a plane
     * parallel to the physical ground at the same height as the angler.
     *
     * <p>
     * To solve for this tangent, we have to employ not one but two right triangles.
     * The first is triangle OAP, with a right angle at A. This is fully defined
     * because we know the vector OP, we know that one angle is a right angle, and
     * we know how long OA is. Using that information, we can solve for angle O. But
     * that's not all we need, because we also need the angle between OP and the
     * aforementioned horizon, which can be calculated with another right triangle
     * that is hopefully intuitive. Adding these two angles together we get our
     * answer.
     *
     * @param angler The vector of the angle changer from the field origin
     * @param target The vector of the target from the field origin
     */
    private void aimVertically(Translation3d angler, Translation3d target) {
        // right triangle spam
        // theta 0 is parallel to the ground and facing the front of the robot
        Translation3d anglerToTarget = target.minus(angler);
        double anglerToTargetAngle1 = Math.atan2(anglerToTarget.getZ(), anglerToTarget.getX());
        // double check this
        double anglerToTargetAngle2 = Math.acos(LimelightConstants.ANGLE_CHANGER_RADIUS / anglerToTarget.getNorm());
        double angleChangerDesiredAngle = radiansEnsureInBounds(anglerToTargetAngle1 + anglerToTargetAngle2);
        double anglerSetpoint = -Math.toDegrees(angleChangerDesiredAngle) + 180;

        m_doubles.set("angle changer radians", angleChangerDesiredAngle);

        angleChanger.setSetpoint(anglerSetpoint);
    }

    /**
     * Ensures angle is between +pi and -pi. If it isn't, it wraps around.
     *
     * <p>
     * As an example, if 4 is inputted, then the function returns -pi + (4 - pi) =
     * -2pi + 4. This is used to convert a value that perhaps goes from 0 to 2pi or
     * perhaps from -2pi to 0 into a value that goes from -pi to pi, which its users
     * prefer.
     *
     * @param angle The angle to transform, if necessary
     *
     * @return The transformed angle, between -pi and pi
     */
    private double radiansEnsureInBounds(double angle) {
        if (angle > -Math.PI && angle < Math.PI) {
            return angle;
        }
        double diff = Math.abs(Math.abs(angle) - Math.PI);
        return Math.PI * -Math.signum(angle) + diff * Math.signum(angle);
    }

    /**
     * This function is a version of the above with reduced functionality. It simply
     * takes in a value that is possibly greater than pi, and if it is, it adds it
     * to negative pi.
     *
     * @param in The angle to process, potentially greater than pi
     *
     * @return The processed angle, between -pi and pi assuming in is less than 2pi
     */
    public static double positiveToPosNeg(double in) {
        if (in > Math.PI) {
            return Math.PI - in;
        }
        return in;
    }

    /**
     * This reverts the above. If the input is between -pi and pi, it returns a
     * value representing the same rotation but bound between 0 and 2pi instead.
     *
     * @param in The angle, bound between -pi and pi
     *
     * @return The angle but bound between 0 and 2pi
     */
    public static double posNegToPositive(double in) {
        if (in < 0) {
            return 2 * Math.PI + in;
        }
        return in;
    }

    @Override
    public void end(final boolean interrupted) {
    }

    /**
     * Determines whether the command should end. If infinite aim is true, that is,
     * it continues to try to aim even after it's already scheduled one
     * {@link LimelightTurnToCommand} and schedules more. This shouldn't happen and
     * this functionality is only for testing;
     * {@code LimelightConstants.INFINITE_AIM} should always be false for this
     * reason. If it is, it checks whether the Limelight has reported two AprilTags
     * seen since the command started.
     *
     * @return Whether the command is done processing
     */
    @Override
    public boolean isFinished() {
        if (LimelightConstants.INFINITE_AIM) {
            return false;
        } else {
            return targetAcquired;
        }
    }
}
