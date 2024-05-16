package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FactoryCommands {
    /**
     * Returns a new {@link Command} that runs an action, stopping when a
     * {@link Supplier} for a boolean returns
     * true. This makes a new composition such that the command running the action
     * is interrupted when isFinished returns true.
     * 
     * @param action       The {@link Runnable} to run
     * @param isFinished   The {@link Supplier} that returns true when this Command
     *                     is to finish
     * @param requirements The {@link Subsystem}s that this Command requires
     *
     * @return A Command that runs <b>action</b> until <b>isFinished</b> returns
     *         true
     */
    public static Command runUntil(Runnable action, Supplier<Boolean> isFinished, Subsystem... requirements) {
        return Commands.run(action, requirements).until(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isFinished.get();
            }
        });
    }

    /**
     * Returns a new command that runs an action once and finishes when a
     * {@link Supplier} for a boolean
     * returns true. This is done with with a
     * {@link edu.wpi.first.wpilibj2.command.ParallelCommandGroup} and
     * {@link edu.wpi.first.wpilibj2.command.WaitUntilCommand}, so that the action
     * is run once and then the command finishes when isFinished returns true.
     *
     * @param action       The {@link Runnable} to run once
     * @param isFinished   The {@link Supplier} that returns true when this Command
     *                     is to finish
     * @param requirements The {@link Subsystem}(s) that this Command requires
     *
     * @return A Command that runs <b>action</b> once and finishes when
     *         <b>isFinished</b> returns
     *         true
     */
    public static Command runOnceUntil(Runnable action, Supplier<Boolean> isFinished, Subsystem... requirements) {
        return Commands.parallel(Commands.runOnce(action, requirements),
                Commands.waitUntil(new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                        return isFinished.get();
                    }
                }));
    }

    /**
     * Returns a new {@link Command} that runs an action once and finishes when a
     * {@link Supplier} for a boolean returns true, running another action and
     * descheduling the Command. This makes a new composition that on schedule, runs
     * start, then queries isFinished constantly and runs action and ends when it
     * returns true.
     *
     * @param start        The {@link Runnable} to run on schedule
     * @param end          The {@link Runnable} to run on ending.
     * @param isFinished   The {@link Supplier} that returns true when this Command
     *                     is to finish
     * @param requirements The {@link Subsystem}s that this Command requires
     *
     * @return A Command that runs <b>action</b> once and finishes when
     *         <b>isFinished</b> returns
     *         true
     */
    public static Command startEndUntil(Runnable start, Runnable end, Supplier<Boolean> isFinished,
            Subsystem... requirements) {
        return Commands.startEnd(start, end, requirements).until(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isFinished.get();
            }
        });
    }
}
