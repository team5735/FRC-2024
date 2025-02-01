// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.dnn.Net;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    DrivetrainSubsystem drivetrain;
    private final StructPublisher<Pose2d> limelightPosePublisher = NetworkTableInstance.getDefault()
            .getTable("telemetry").getStructTopic("pose estimation", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> limelightMt2Publisher = NetworkTableInstance.getDefault()
            .getTable("telemetry").getStructTopic("mt2", Pose2d.struct).publish();
    @SuppressWarnings("unused")
    private double driftEstimateTicks;

    // Initializes the vision subsystem
    public VisionSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        seedPigeon();
    }

    private LimelightHelpers.PoseEstimate botPoseEstimate;

    private void seedPigeon() {
        botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(null);
        if (botPoseEstimate == null) {
            return;
        }
        double limelightRotation = botPoseEstimate.pose.getRotation().getRadians();
        drivetrain.getPigeon2().setYaw(limelightRotation);
    }

    public Command getSeedPigeon() {
        return new SequentialCommandGroup(
                runOnce(() -> SmartDashboard.putBoolean("pigeon resetting", true)),
                run(() -> seedPigeon()).until(() -> botPoseEstimate != null && botPoseEstimate.tagCount > 0),
                runOnce(() -> SmartDashboard.putBoolean("pigeon resetting", false)));
    }

    private void keepDriftInCheck() {
        driftEstimateTicks += 1;
        // TODO: is this even a problem that we need to correct for?
    }

    private Pose2d updateVisionMeasurement() {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (mt2 == null) {
            // failed to get mt2
            SmartDashboard.putNumber("poseestimator_status", -1);
            return null;
        }
        if (mt2.tagCount == 0) {
            // no tags
            SmartDashboard.putNumber("poseestimator_status", -2);
            return null;
        }
        SmartDashboard.putNumber("poseestimator_status", 0);

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        drivetrain.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);

        return mt2.pose;
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(null,
                this.drivetrain.getEstimatedPosition().getRotation().getDegrees(), 0, 0,
                0, 0, 0);

        Pose2d mt2 = updateVisionMeasurement();
        if (mt2 != null) {
            this.limelightMt2Publisher.set(mt2);
        }
        this.limelightPosePublisher.set(drivetrain.getEstimatedPosition());

        keepDriftInCheck();
    }
}
