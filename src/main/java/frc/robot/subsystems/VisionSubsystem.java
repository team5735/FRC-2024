// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        // seedPigeon();
    }

    private void seedPigeon() {
        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(null);
        boolean hasTarget = LimelightHelpers.getTV(null);
        if (pose == null || !hasTarget) {
            return;
        }
        double rot = pose.getRotation().getDegrees();
        System.out.println("setting yaw to: " + rot);
        drivetrain.getPigeon2().setYaw(rot);
    }

    public Command getSeedPigeon() {
        return new SequentialCommandGroup(
                runOnce(() -> SmartDashboard.putBoolean("pigeon resetting", true)),
                runEnd(() -> seedPigeon(), () -> SmartDashboard.putBoolean("pigeon resetting", false)));
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

    // all in deg
    NTDoubleSection telemetry_doubles = new NTDoubleSection("test_telem_doubles", "mt1_rz", "mt2_rz", "pigeon",
            "poseest", "averagedMt1");

    Pose2d lastEstPos = null;
    int ticks = 0;

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

        telemetry_doubles.set("mt1_rz", LimelightHelpers.getBotPose2d_wpiBlue(null).getRotation().getDegrees());
        telemetry_doubles.set("mt2_rz",
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(null).pose.getRotation().getDegrees());
        telemetry_doubles.set("pigeon", drivetrain.getPigeon2().getRotation2d().getDegrees());
        telemetry_doubles.set("poseest", drivetrain.getEstimatedPosition().getRotation().getDegrees());

        if (!LimelightHelpers.getTV(null)) {
            lastEstPos = null;
        } else if (lastEstPos == null) {
            lastEstPos = drivetrain.getEstimatedPosition();
        } else if (Math.abs(drivetrain.getEstimatedPosition().getRotation().getDegrees()
                - lastEstPos.getRotation().getDegrees()) < LimelightConstants.DRIVETRAIN_STILL_THRESHOLD // we're still
                                                                                                         // enough
                && ticks >= 10) { // it's been long enough
            lastEstPos = drivetrain.getEstimatedPosition();
            ticks = 0;
            drivetrain.getPigeon2().setYaw(LimelightHelpers.getBotPose2d(null).getRotation().getDegrees());
        }
        ticks++;
    }
}
