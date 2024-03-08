// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
    private Watchdog m_watchdog = new Watchdog(0.02, () -> {
    });

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
        m_watchdog.reset();
        LimelightTarget_Fiducial[] targets = m_limelight.getTargetedFiducials();
        m_watchdog.addEpoch("get fiducial info");
        if (targets.length < 2) {
            System.out.println("spinning");
            m_drivetrain.drive(1);
            return;
        }
        m_targetAcquired = true;
        m_watchdog.addEpoch("conclude that a target has been acquired");

        // coordinate system for field-oriented limelight targeting: x along long side
        // with positive towards red alliance, y along short side such that positive is
        // facing the long side that has red on the right and blue on the left, z up,
        // positive theta is counterclockwise and theta 0 is facing the red alliance
        // speaker.

        Translation3d hoodPos = getHoodPos();
        Pose3d currentRobotPose = m_limelight.getBotPose3d();
        m_drivetrain.addVisionMeasurement(currentRobotPose.toPose2d(), Timer.getFPGATimestamp());
        m_drivetrain.seedFieldRelative(currentRobotPose.toPose2d());
        m_watchdog.addEpoch("fieldRelative seeded");

        checkBotCanAim(currentRobotPose.getTranslation(), hoodPos);
        m_watchdog.addEpoch("checked bot can aim");

        Translation3d robotToHood = hoodPos.minus(currentRobotPose.getTranslation());
        aimHorizontally(robotToHood, currentRobotPose.toPose2d());
        m_watchdog.addEpoch("aimed horizontally");

        Translation3d robotPosTranslation3d = new Translation3d(currentRobotPose.getX(), currentRobotPose.getY(), 0);
        Translation3d angleChangerPosition = robotPosTranslation3d.plus(LimelightConstants.ANGLE_CHANGER_POS);
        aimVertically(angleChangerPosition, getHoodPos());
        m_watchdog.addEpoch("aimed vertically");

        SmartDashboard.putNumber("llv2_hoodDst", robotToHood.getNorm());
        m_watchdog.disable();
        m_watchdog.printEpochs();
    }

    // checks to see if the robot is within reasonable shooting range of the target
    private void checkBotCanAim(Translation3d robotPosition, Translation3d targetPosition) {
        boolean botIsCloseEnough = robotPosition.toTranslation2d()
                .getDistance(targetPosition.toTranslation2d()) < LimelightConstants.BOT_SHOOTING_DISTANCE;
        if (!botIsCloseEnough) {
            SmartDashboard.putNumber("llv2_can'tAim",
                    robotPosition.toTranslation2d().getDistance(getHoodPos().toTranslation2d()));
            System.out.println("moving towards the hood");
            Translation2d robotToTarget = targetPosition.toTranslation2d().minus(robotPosition.toTranslation2d());
            Translation2d desiredVelocity = robotToTarget.div(robotToTarget.getNorm()) // normalize the vector
                    .times(LimelightConstants.DRIVETRAIN_MOVEMENT_SPEED); // set magnitude to allowed drivetrain
                                                                          // movement speed
            // m_drivetrain.drive(desiredVelocity);
            SmartDashboard.putNumber("llv2_moveXCorrection", desiredVelocity.getX());
            SmartDashboard.putNumber("llv2_moveYCorrection", desiredVelocity.getY());
            return;
        }
    }

    private void aimHorizontally(Translation3d currentRobotPoseToTarget, Pose2d robotPoseInField) {
        double drivetrainDesiredAngle = Math.atan2(currentRobotPoseToTarget.getY(), currentRobotPoseToTarget.getX())
                + Math.PI - 0.01; // don't question it
        if (drivetrainDesiredAngle > Math.PI) {
            drivetrainDesiredAngle -= 2 * Math.PI;
        }

        // CommandScheduler.getInstance().schedule(new TurnToCommand(m_drivetrain,
        // drivetrainDesiredAngle));
        CommandScheduler.getInstance()
                .schedule(new PIDCommand(
                        new PIDController(LimelightConstants.TURN_P, LimelightConstants.TURN_I,
                                LimelightConstants.TURN_D),
                        () -> m_drivetrain.getRotation3d().getZ(), drivetrainDesiredAngle,
                        (double output) -> m_drivetrain.drive(output), m_drivetrain));
    }

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
        m_angler.setSetpoint(angleChangerDesiredAngle);

        SmartDashboard.putNumber("llv2_anglerRad", angleChangerDesiredAngle);
        SmartDashboard.putNumber("llv2_anglerDeg", Math.toDegrees(angleChangerDesiredAngle));
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
