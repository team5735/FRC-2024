// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightConstants;
import frc.robot.libraries.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/** An example command that uses an example subsystem. */
public class LimelightAimCommandV2 extends Command {
    private LimelightSubsystem m_limelight;
    private DrivetrainSubsystem m_drivetrain;
    private AngleSubsystem m_angler;
    private boolean m_targetAcquired = false;
    private Alliance m_alliance;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public LimelightAimCommandV2(final LimelightSubsystem limelight, final DrivetrainSubsystem drivetrain,
            final AngleSubsystem angler) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(limelight, drivetrain);
        m_limelight = limelight;
        m_drivetrain = drivetrain;
        m_angler = angler;
        Optional<Alliance> ally = DriverStation.getAlliance();
        m_alliance = ally.isPresent() ? ally.get() : Alliance.Red;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    private Translation3d getHoodPos() {
        Translation3d hoodPos;
        if (m_alliance == Alliance.Red) {
            hoodPos = LimelightConstants.HOOD_POS;
        } else {
            Translation3d hoodPosTmp = LimelightConstants.HOOD_POS;
            hoodPos = new Translation3d(-hoodPosTmp.getX(), hoodPosTmp.getY(), hoodPosTmp.getZ());
        }
        return hoodPos;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        LimelightTarget_Fiducial[] targets = m_limelight.getTargetedFiducials();
        if (targets.length < 2) {
            System.out.println("spinning");
            m_drivetrain.drive(0, 0, 1);
            return;
        }
        m_targetAcquired = true;
        System.out.println("target acquired");

        // coordinate system for field-oriented limelight targeting: x along long side
        // with positive towards red alliance, y along short side with positive facing
        // opposite the side that theta zero faces, z up, positive theta is
        // counterclockwise and theta 0 is facing the red alliance speaker.

        Translation3d hoodPos = getHoodPos();
        Pose3d currentRobotPose = m_limelight.getBotPose3d();
        m_drivetrain.seedFieldRelative(currentRobotPose.toPose2d());

        checkBotCanAim(currentRobotPose.getTranslation(), hoodPos);

        Translation3d robotToHood = hoodPos.minus(currentRobotPose.getTranslation());
        aim(robotToHood);
    }

    // checks to see if the robot is within reasonable shooting range of the target
    private void checkBotCanAim(Translation3d robotPosition, Translation3d targetPosition) {
        boolean botIsCloseEnough = robotPosition.toTranslation2d()
                .getDistance(targetPosition.toTranslation2d()) < LimelightConstants.BOT_SHOOTING_DISTANCE;
        if (!botIsCloseEnough) {
            System.out.println("bot aim distance check failed"
                    + robotPosition.toTranslation2d().getDistance(getHoodPos().toTranslation2d()));
            System.out.println("moving towards the hood");
            Translation2d robotToTarget = targetPosition.toTranslation2d().minus(robotPosition.toTranslation2d());
            Translation2d desiredVelocity = robotToTarget.div(robotToTarget.getNorm()) // normalize the vector
                    .times(LimelightConstants.DRIVETRAIN_MOVEMENT_SPEED); // set magnitude to allowed drivetrain
                                                                          // movement speed
            // TODO: check coordinate systems (could be okay due to seedFieldRelative
            // above?)
            m_drivetrain.drive(desiredVelocity);
            // TODO: look into drivetrain.addVisionMeasurement
            return;
        }
    }

    private void aim(Translation3d currentRobotPoseToTarget) {
        Translation3d angleChangerToHood = currentRobotPoseToTarget.minus(LimelightConstants.ANGLE_CHANGER_POS);
        double angleChangerDesiredAngle = Math.atan2(angleChangerToHood.getZ(),
                new Translation2d(angleChangerToHood.getX(), angleChangerToHood.getY()).getNorm());
        m_angler.setSetpoint(angleChangerDesiredAngle);
        double drivetrainDesiredAngle = Math.atan2(currentRobotPoseToTarget.getY(), currentRobotPoseToTarget.getX());
        double drivetrainDeltaTheta = drivetrainDesiredAngle;

        System.out.println("distance to hood: " + angleChangerToHood.getNorm());
        System.out.println("angle changer desired angle: " + angleChangerDesiredAngle);
        System.out.println("delta theta for drivetrain: " + drivetrainDeltaTheta);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return LimelightConstants.INFINITE_AIM ? false : m_targetAcquired;
    }
}
