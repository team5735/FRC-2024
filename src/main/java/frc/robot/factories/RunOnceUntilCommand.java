package frc.robot.factories;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.FactoryCommands;

/**
 * RunOnceUntilCommand is a Command that runs a specific Runnable in its
 * initialize method, and returns the result of a Supplier<Boolean> for its
 * isFinished method. The varargs requirements are passed directly to
 * addRequirements. Nothing else is done with the requirements. The intention of
 * this class is not to be used on its own; instead, please use runUntil in
 * {@link FactoryCommands}.
 */
public class RunOnceUntilCommand extends Command {
    private final Runnable m_action;
    private final Supplier<Boolean> m_isFinished;

    /**
     * Creates a new RunOnceUntilCommand, which runs action once and finishes when
     * isFinished returns true, requiring requirements.
     */
    public RunOnceUntilCommand(final Runnable action, final Supplier<Boolean> isFinished,
            final Subsystem... requirements) {
        addRequirements(requirements);

        m_action = action;
        m_isFinished = isFinished;
    }

    @Override
    public void initialize() {
        m_action.run();
    }

    /**
     * Runs the action passed to the constructor.
     */
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_isFinished.get();
    }
}
