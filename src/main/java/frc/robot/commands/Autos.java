// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.limelight.LimelightAimCommandV2;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static Command auto(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain, CANdleSubsystem candle) {
        return Commands.sequence(candle.colorAuto(), new LimelightAimCommandV2(limelight), candle.colorReady());
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
