// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AllianceSwitcher;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTBooleanSection;
import frc.robot.util.NTDoubleSection;

/** An example command that uses an example subsystem. */
public class LimelightAimCommand extends Command {
    private DrivetrainSubsystem m_drivetrain;
    private boolean m_targetAcquired = false;
    private Watchdog m_watchdog = new Watchdog(0.02, () -> {
    });

    private Supplier<Double> measurementGetter;

    private final NTDoubleSection m_doubles = new NTDoubleSection("limelight", "current rotation", "hood distance",
            "cannot aim distance", "drivetrain speed x", "drivetrain speed y", "desired drivetrain offset", "setpoint",
            "hood vector x", "hood vector y", "angle changer radians", "angler setpoint", "bot x", "bot y");

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
    public LimelightAimCommand(final DrivetrainSubsystem drivetrain,
            final Supplier<Double> measurementSupplier) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
        m_targetAcquired = false;
        measurementGetter = measurementSupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_watchdog.reset();

        var botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(null);
        Pose2d currentRobotPose = botPoseEstimate.pose;
        m_watchdog.addEpoch("get bot pose");

        m_booleans.set("aiming", true);
        m_targetAcquired = true;
        m_watchdog.addEpoch("a target has been acquired");

        // coordinate system for field-oriented limelight targeting: x along long side
        // with positive towards red alliance, y along short side such that positive is
        // facing the long side that has red on the right and blue on the left, z up,
        // positive theta is counterclockwise and theta 0 is facing the red alliance
        // speaker.

        m_doubles.set("bot x", currentRobotPose.getX());
        m_doubles.set("bot y", currentRobotPose.getY());

        Translation3d hoodPos = m_hoodPos;
        checkBotCanAim(currentRobotPose.getTranslation(), hoodPos);
        m_watchdog.addEpoch("checked bot can aim");

        Translation2d robotToHood = hoodPos.toTranslation2d().minus(currentRobotPose.getTranslation());
        aimHorizontally(robotToHood, currentRobotPose.getRotation().getRadians());
        m_doubles.set("current rotation", currentRobotPose.getRotation().getRadians());
        m_watchdog.addEpoch("aimed horizontally");

        Translation3d robotPosTranslation3d = new Translation3d(currentRobotPose.getX(), currentRobotPose.getY(), 0);
        double robotRotation = currentRobotPose.getRotation().getRadians();
        Translation3d angleChangerPosition = robotPosTranslation3d.plus(LimelightConstants.ANGLE_CHANGER_OFFSET
                .rotateBy(new Rotation3d(0, 0, robotRotation)));
        aimVertically(angleChangerPosition, hoodPos);
        m_watchdog.addEpoch("aimed vertically");

        m_doubles.set("hood distance", robotToHood.getNorm());
        m_watchdog.disable();
        m_watchdog.printEpochs();
    }

    // checks to see if the robot is within reasonable shooting range of the target
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
        return;
    }

    private void aimHorizontally(Translation2d currentRobotPoseToTarget, double curRobotRot) {
        double drivetrainAngleToTarget = Math.atan2(currentRobotPoseToTarget.getY(), currentRobotPoseToTarget.getX());
        double target = radiansEnsureInBounds(drivetrainAngleToTarget - curRobotRot);

        m_doubles.set("desired drivetrain offset", drivetrainAngleToTarget);
        m_doubles.set("setpoint", measurementGetter.get() + target);
        m_doubles.set("hood vector x", currentRobotPoseToTarget.getX());
        m_doubles.set("hood vector y", currentRobotPoseToTarget.getY());

        m_drivetrain.setRotationTarget(target);
    }

    // Sometimes angles go past +pi or -pi. This function returns an angle that
    // represents the same rotation, but is actually within the bounds set. Probably
    // not needed - I think PIDControllers do this automatically - but I was bored.
    private double radiansEnsureInBounds(double angle) {
        if (angle > -Math.PI && angle < Math.PI) {
            return angle;
        }
        double diff = Math.abs(Math.abs(angle) - Math.PI);
        return Math.PI * -Math.signum(angle) + diff * Math.signum(angle);
    }

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
        m_doubles.set("angler setpoint", anglerSetpoint);

        // m_angleChanger.setSetpoint(anglerSetpoint);
    }

    public static double positiveToPosNeg(double in) {
        if (in > Math.PI) {
            return -(in - Math.PI);
        }
        return in;
    }

    public static double posNegToPositive(double in) {
        if (in < 0) {
            return Math.PI + (-in);
        }
        return in;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        m_drivetrain.setRotationTarget(null);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (LimelightConstants.INFINITE_AIM) {
            return false;
        } else {
            return m_targetAcquired;
        }
    }
}
