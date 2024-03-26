// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConstants;

public class VisionSubsystem extends SubsystemBase {
    private VisionResults results = new VisionResults();
    private VisionIO io;

    public VisionSubsystem(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateResults(results);
    }

    public Command blinkLEDs() {
        return Commands.repeatingSequence(
                Commands.runOnce(() -> io.ledsOn()),
                Commands.waitSeconds(LimelightConstants.BLINK_TIME),
                Commands.runOnce(() -> io.ledsOff()),
                Commands.waitSeconds(LimelightConstants.BLINK_TIME));
    }
}
