// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
    private ClimberSubsystem m_subsystem;
    private Supplier<Double> m_rightArmMovement;
    private Supplier<Double> m_leftArmMovement;

    /** Creates a new ClimbCommand. */
    public ClimbCommand(final ClimberSubsystem subsystem, Supplier<Double> rightArmMovement,
            Supplier<Double> leftArmMovement) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        m_subsystem = subsystem;
        m_rightArmMovement = rightArmMovement;
        m_leftArmMovement = leftArmMovement;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.left(m_leftArmMovement.get());
        m_subsystem.right(m_rightArmMovement.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
  }
}
