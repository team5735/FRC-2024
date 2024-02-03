// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class LimelightSubsystem extends SubsystemBase {
    private boolean m_staleLLData;
    private LimelightHelpers.Results m_targetingResults;

    /** Creates a new ExampleSubsystem. */
    public LimelightSubsystem() {}

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(()
                           -> {
                               /* one-time action goes here */
                           });
    }

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

    public Pose3d getBotPose3d() {
        tryFetchLL();
        return m_targetingResults.getBotPose3d();
    }

    public double getLatency() {
        return m_targetingResults.latency_capture +
            m_targetingResults.latency_pipeline +
            m_targetingResults.latency_jsonParse;
    }

    private void tryFetchLL() {
        if (!m_staleLLData) {
            LimelightResults latestResults =
                LimelightHelpers.getLatestResults("");
            m_targetingResults = latestResults.targetingResults;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        m_staleLLData = true;
    }
}
