// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final DoubleArraySubscriber m_poseSubscriber;

    private final String m_name;
    private final NetworkTable m_table;

    private VisionResults m_results = new VisionResults();

    public VisionSubsystem(String name) {
        m_name = name;
        m_table = NetworkTableInstance.getDefault().getTable(m_name);
        m_poseSubscriber = m_table.getDoubleArrayTopic("pose").subscribe(new double[6]);
    }

    @Override
    public void periodic() {
    }
}
