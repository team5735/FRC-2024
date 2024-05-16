package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

/**
 * A simple {@link Command}, intended to spin up the passed-in
 * {@link ShooterTopSubsystem} and {@link ShooterBottomSubsystem}, and thusly
 * end. This is intended for use in composed {@link Commmand}s, to allow for
 * sequentially reaching a desired speed before shooting.
 * 
 * @author Jacoby
 */
public class ShooterSpinUpCommand extends Command {
    ShooterTopSubsystem m_subsystemTop;
    ShooterBottomSubsystem m_subsystemBottom;
    double m_setpoint_top, m_setpoint_bottom;

    /**
     * Constructs a new ShooterSpinUpCommand with the passed-in values.
     * 
     * @param st   - the {@link ShooterTopSubsystem} to spin up to a desired speed.
     * @param sb   - the {@link ShooterBottomSubsystem} to spin up to a desired
     *             speed.
     * @param trpm - the desired speed for {@code st}, in rotations per minute.
     * @param brpm - the desired speed for {@code sb}, in rotations per minute.
     */
    public ShooterSpinUpCommand(ShooterTopSubsystem st, ShooterBottomSubsystem sb, double trpm, double brpm) {
        m_subsystemTop = st;
        m_subsystemBottom = sb;
        m_setpoint_top = trpm;
        m_setpoint_bottom = brpm;
    }

    /**
     * Called when the ShooterSpinUpCommand is first scheudled, will set the
     * setpoints on both the {@link ShooterTopSubsystem} and the
     * {@link ShooterBottomSubsystem} via their {@code setSetpointRPM()} methods.
     */
    @Override
    public void initialize() {
        m_subsystemTop.setSetpointRPM(m_setpoint_top);
        m_subsystemBottom.setSetpointRPM(m_setpoint_bottom);
    }

    /**
     * Called when the ShooterSpinUpCommand is either interrupted or ends naturally.
     * <p>
     * Will end the the {@link ShooterTopSubsystem} and the
     * {@link ShooterBottomSubsystem} via their {@code stop()} methods, but ONLY if
     * another {@link Command} has interrupted this one.
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_subsystemTop.stop();
            m_subsystemBottom.stop();
        }
    }

    /**
     * Constantly queried boolean value, will deschedule the command when true.
     * 
     * @return whether or not both the {@link ShooterTopSubsystem} and the
     *         {@link ShooterBottomSubsystem} are spun up, as determined by their
     *         {@code isSpunUp()} methods,
     */
    @Override
    public boolean isFinished() {
        return m_subsystemTop.isSpunUp() && m_subsystemBottom.isSpunUp();
    }
}
