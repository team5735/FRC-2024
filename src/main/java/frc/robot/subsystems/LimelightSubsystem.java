// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConstants;
import frc.robot.libraries.LimelightHelpers;
import frc.robot.libraries.LimelightHelpers.LimelightResults;
import frc.robot.libraries.LimelightHelpers.LimelightTarget_Fiducial;

public class LimelightSubsystem extends SubsystemBase {
    private boolean m_staleLLData = false;
    private LimelightHelpers.Results m_targetingResults;
    private static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    /** Creates a new ExampleSubsystem. */
    public LimelightSubsystem() {
    }

    private static DoublePublisher ledModePublisher = limelightTable.getDoubleTopic("ledMode").publish();

    /**
     * An example method querying a boolean state of the subsystem (for example,
     * a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean hasTargetedApriltag() {
        // Query some boolean state, such as a digital sensor.
        tryFetchLL();
        return m_targetingResults.targets_Fiducials.length > 0;
    }

    public LimelightTarget_Fiducial[] getTargetedFiducials() {
        tryFetchLL();
        return m_targetingResults.targets_Fiducials;
    }

    public LimelightTarget_Fiducial getMainTargetedFiducial() {
        tryFetchLL();
        // lazy (no better way to get main fiducial that i can tell) (terrible
        // library)
        return m_targetingResults.targets_Fiducials[0];
    }

    public Pose2d getBotPose2d() {
        tryFetchLL();
        return m_targetingResults.getBotPose2d();
    }

    public static void ledsOn() {
        ledModePublisher.set(3);
    }

    public static void ledsOff() {
        ledModePublisher.set(1);
    }

    public Pose3d getBotPose3d() {
        tryFetchLL();
        return m_targetingResults.getBotPose3d();
    }

    public double getLatency() {
        return m_targetingResults.latency_capture +
                m_targetingResults.latency_pipeline +
                m_targetingResults.latency_jsonParse;
    }

    public static Command blinkLedsOnce() {
        return Commands.sequence(
                Commands.runOnce(() -> ledsOn()),
                Commands.runOnce(() -> Commands.waitSeconds(LimelightConstants.BLINK_TIME)),
                Commands.runOnce(() -> ledsOff()),
                Commands.runOnce(() -> Commands.waitSeconds(LimelightConstants.BLINK_TIME)));
    }

    private void tryFetchLL() {
        if (m_staleLLData) {
            LimelightResults latestResults = LimelightHelpers.getLatestResults("");
            m_targetingResults = latestResults.targetingResults;
        }
    }

    public static Command blinkLeds(int numberOfTimes) {
        Command currentlyBuiltCommand = Commands.none();
        for (int i = 0; i < numberOfTimes; i++) {
            currentlyBuiltCommand = currentlyBuiltCommand.andThen(blinkLedsOnce());
        }
        return currentlyBuiltCommand.andThen(Commands.runOnce(() -> ledsOff()));
    }

    public Command seedSwerveDrivetrain(DrivetrainSubsystem drivetrain) {
        return Commands.runOnce(() -> drivetrain.seedFieldRelative(), drivetrain, this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_staleLLData = true;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
