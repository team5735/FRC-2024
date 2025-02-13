// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTDoubleSection;

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
    }

    @SuppressWarnings("unused")
    private void oldSeedPigeon() {
        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(null);
        boolean hasTarget = LimelightHelpers.getTV(null);
        if (pose == null || !hasTarget) {
            return;
        }
        double rot = pose.getRotation().getDegrees();
        System.out.println("setting yaw to: " + rot);
        drivetrain.getPigeon2().setYaw(rot);
    }

    Pose2d lastEstPos = null;
    int ticks = 0;

    /**
     * If no april-tag data -> null last-pos.
     * If null last-pos -> set it to the current position.
     * Otherwise, if we're still enough and it's been long enough since the last
     * update:
     * - update the last estimated position
     * - set the pigeon
     */
    private void seedPigeon() {
        if (lastEstPos == null) {
            lastEstPos = drivetrain.getEstimatedPosition();
        } else if (LimelightHelpers.getTV(null) &&
                Math.abs(drivetrain.getEstimatedPosition().getRotation().getDegrees()
                        - lastEstPos.getRotation().getDegrees()) < LimelightConstants.DRIVETRAIN_STILL_THRESHOLD

                && ticks >= LimelightConstants.TICKS_BETWEEN_PIGEON_UPDATES) {
            lastEstPos = drivetrain.getEstimatedPosition();
            ticks = 0;

            Rotation2d rot = LimelightHelpers.getBotPose2d(null).getRotation();
            drivetrain.getPigeon2().setYaw(rot.getDegrees());
            System.out.println("set pigeon yaw to deg " + rot.getDegrees());
        } else {
            lastEstPos = drivetrain.getEstimatedPosition();
        }
        ticks++;
    }

    public Command getSeedPigeon() {
        return run(() -> seedPigeon());
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

    // all in deg
    NTDoubleSection telemetry_doubles = new NTDoubleSection("test_telem_doubles", "mt1_rz", "mt2_rz", "pigeon",
            "poseest", "averagedMt1");

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

        telemetry_doubles.set("mt1_rz", LimelightHelpers.getBotPose2d_wpiBlue(null).getRotation().getDegrees());
        telemetry_doubles.set("mt2_rz",
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(null).pose.getRotation().getDegrees());
        telemetry_doubles.set("pigeon", drivetrain.getPigeon2().getRotation2d().getDegrees());
        telemetry_doubles.set("poseest", drivetrain.getEstimatedPosition().getRotation().getDegrees());

        seedPigeon();
    }
}
