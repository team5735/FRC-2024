package frc.robot.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

/**
 * A command that uses a PIDController to control an output. The difference
 * between this and {@link PIDCommand} is that this command re-creates the
 * PIDController every initialization.
 */
public class TunablePIDCommand extends Command {
    /** PID controller. */
    protected PIDController controller;

    /** Measurement getter. */
    protected Supplier<Double> measurement;

    /** Setpoint getter. */
    protected Supplier<Double> setpoint;

    /** PID controller output consumer. */
    protected Consumer<Double> useOutput;

    protected TunableNumber p, i, d;

    /**
     * Creates a new PIDCommand, which controls the given output with a
     * PIDController.
     *
     * @param controller        the controller that controls the output.
     * @param measurementSource the measurement of the process variable
     * @param setpointSource    the controller's setpoint
     * @param useOutput         the controller's output
     * @param requirements      the subsystems required by this command
     */
    public TunablePIDCommand(
            Supplier<Double> measurementSource,
            Supplier<Double> setpointSource,
            Consumer<Double> useOutput,
            String name,
            Subsystem... requirements) {
        requireNonNullParam(measurementSource, "measurementSource", "PIDCommand");
        requireNonNullParam(setpointSource, "setpointSource", "PIDCommand");
        requireNonNullParam(useOutput, "useOutput", "PIDCommand");

        this.useOutput = useOutput;
        measurement = measurementSource;
        setpoint = setpointSource;
        addRequirements(requirements);

        p = new TunableNumber("tunable_pid_commands", name + "_p", 0);
        i = new TunableNumber("tunable_pid_commands", name + "_i", 0);
        d = new TunableNumber("tunable_pid_commands", name + "_d", 0);
    }

    /**
     * Creates a new PIDCommand, which controls the given output with a
     * PIDController.
     *
     * @param controller        the controller that controls the output.
     * @param measurementSource the measurement of the process variable
     * @param setpoint          the controller's setpoint
     * @param useOutput         the controller's output
     * @param requirements      the subsystems required by this command
     */
    public TunablePIDCommand(
            Supplier<Double> measurementSource,
            double setpoint,
            Consumer<Double> useOutput,
            String name,
            Subsystem... requirements) {
        this(measurementSource, () -> setpoint, useOutput, name, requirements);
    }

    @Override
    public void initialize() {
        controller = new PIDController(p.get(), i.get(), d.get());
        controller.setTolerance(Constants.TOLERANCE);
    }

    @Override
    public void execute() {
        useOutput.accept(
                controller.calculate(measurement.get(), setpoint.get()));
    }

    @Override
    public void end(boolean interrupted) {
        useOutput.accept(0.0);
    }

    @Override
    public boolean isFinished() {
        return this.controller.atSetpoint();
    }

    /**
     * Returns the PIDController used by the command.
     *
     * @return The PIDController
     */
    public PIDController getController() {
        return controller;
    }
}
