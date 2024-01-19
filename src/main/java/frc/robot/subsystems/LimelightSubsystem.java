// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class LimelightSubsystem extends SubsystemBase {
    private boolean m_staleLLData;
    private LimelightTarget_Fiducial[] m_targetFiducials;
    /* currently unused */
    private double m_targetingLatency;
    private Pose3d m_guessedPosition;

    /** Creates a new ExampleSubsystem. */
    public LimelightSubsystem() {}

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public CommandBase exampleMethodCommand() {
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
        return m_targetFiducials.length > 0;
    }

    public LimelightTarget_Fiducial[] getTargetedFiducials() {
        tryFetchLL();
        return m_targetFiducials;
    }

    public Pose3d getGuessedPosition() {
        tryFetchLL();
        return m_guessedPosition;
    }

    public double getLatency() { return m_targetingLatency; }

    private void tryFetchLL() {
        if (!m_staleLLData) {
            LimelightResults latestResults =
                LimelightHelpers.getLatestResults("");
            LimelightHelpers.Results res = latestResults.targetingResults;
            m_targetFiducials = res.targets_Fiducials;
            m_targetingLatency = res.latency_capture + res.latency_pipeline +
                                 res.latency_jsonParse;
            m_guessedPosition = res.getBotPose3d();
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
