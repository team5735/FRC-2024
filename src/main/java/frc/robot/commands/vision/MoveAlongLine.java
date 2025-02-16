// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Line;

public class MoveAlongLine extends Command {
    private Supplier<Double> control;
    private Supplier<Line> lineSupplier;
    private Line line;
    private DrivetrainSubsystem drivetrain;

    public MoveAlongLine(Supplier<Double> control, Supplier<Line> lineSupplier, DrivetrainSubsystem drivetrain) {
        this.control = control;
        this.lineSupplier = lineSupplier;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.line = lineSupplier.get();
    }

    @Override
    public void execute() {
        Translation2d vector = this.line.getVectorAlongLine();
        double movement = this.control.get();
        vector = vector.times(movement);
        this.drivetrain.drive(vector);
    }
}
