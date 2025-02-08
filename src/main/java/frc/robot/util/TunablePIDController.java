package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

/**
 * A command that uses a PIDController to control an output. The difference
 * between this and {@link PIDCommand} is that this command re-creates the
 * PIDController every initialization.
 */
public class TunablePIDController {
    /** PID controller. */
    protected PIDController controller;

    protected TunableNumber p, i, d;

    private NTDoubleSection doubles;

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
    public TunablePIDController(
            String name,
            Subsystem... requirements) {

        p = new TunableNumber("tunable_pid_commands", name + "_p", Constants.PID_P);
        i = new TunableNumber("tunable_pid_commands", name + "_i", Constants.PID_I);
        d = new TunableNumber("tunable_pid_commands", name + "_d", Constants.PID_D);

        doubles = new NTDoubleSection(name + " pid", "setpoint", "output", "measurement", "p", "i", "d");
    }

    public void setup(double setpoint) {
        controller = new PIDController(p.get(), i.get(), d.get());
        doubles.set("p", p.get());
        doubles.set("i", i.get());
        doubles.set("d", d.get());

        controller.setSetpoint(setpoint);
        doubles.set("setpoint", setpoint);
        controller.setTolerance(Constants.TOLERANCE);
    }

    /**
     * Runs the PID calculation.
     * 
     * <p>
     * Outputs 0 if the controller is at the setpoint.
     * 
     * @param measurement
     * @return The controller output, or zero if atSetpoint.
     */
    public double calculate(double measurement) {
        if (atSetpoint()) {
            return 0;
        }

        double value = controller.calculate(measurement);
        doubles.set("measurement", measurement);
        doubles.set("output", value);
        return value;
    }

    /**
     * @return Whether the PIDController is within tolerance of the setpoint.
     */
    public boolean atSetpoint() {
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
