package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;

/**
 * This class represents a ShooterBottomSubsystem. It utilizes a sequence of
 * compliant wheels attatched to an axle driven by an {@link TalonFX}. The Talon
 * is controlled via a {@link PIDController} and a
 * {@link SimpleMotorFeedforward} for accurate and precise speed management.
 * 
 * @author Jacoby (with help for Commands)
 */
public class ShooterBottomSubsystem extends SubsystemBase {
    private PIDController m_pid_bottom;
    private SimpleMotorFeedforward m_feedForward_bottom;
    private double m_setpoint;

    private final TalonFX m_talon_bottom = new TalonFX(Constants.SHOOTER_MOTOR_BOTTOM_ID);

    /**
     * Creates a new ShooterBottom Subsystem, setting the motor to coast mode and
     * initizalizing PID and FF values via {@code updateProportions()}
     */
    public ShooterBottomSubsystem() {
        m_talon_bottom.setNeutralMode(NeutralModeValue.Coast);

        m_pid_bottom = new PIDController(0, 0, 0);
        m_feedForward_bottom = new SimpleMotorFeedforward(0, 0);

        updateProportions();
    }

    /**
     * Gets values from {@link ShooterConstants} for the {@link PIDController} and
     * the
     * {@link SimpleMotorFeedforward}. Then, {@code m_pid_bottom} and {@code m_feedForward_bottom}
     * are reconstructed based on the values acquired from {@link ShooterConstants}.
     *
     * <p>
     * Uses the following values:
     * <ul>
     * <li>{@code BOTTOM_KP}
     * <li>{@code BOTTOM_KI}
     * <li>{@code BOTTOM_KD}
     * <li>{@code BOTTOM_KS}
     * <li>{@code BOTTOM_KV}
     * </ul>
     */
    public void updateProportions() {
        double bkp = ShooterConstants.BOTTOM_KP;
        double bki = ShooterConstants.BOTTOM_KI;
        double bkd = ShooterConstants.BOTTOM_KD;

        double bks = ShooterConstants.BOTTOM_KS;
        double bkv = ShooterConstants.BOTTOM_KV;

        m_pid_bottom.setPID(bkp, bki, bkd);
        m_feedForward_bottom = new SimpleMotorFeedforward(bks, bkv);
    }

    /**
     * Periodic method called by the CommandScheduler, changes the PID and FF values
     * via {@code UpdateProportions()} and logs values to {@link SmartDashboard}.
     */
    @Override
    public void periodic() {
        updateProportions();

        SmartDashboard.putNumber("shootBottomOutput", Math.abs(getMotorRPM()));
        SmartDashboard.putNumber("shootBottomPIDError", Math.abs(m_pid_bottom.getPositionError()));
        SmartDashboard.putNumber("shootBottomSetpoint", m_setpoint);
        SmartDashboard.putNumber("shootTopAmps", m_talon_bottom.getStatorCurrent().getValueAsDouble());
    }

    /**
     * A method to send voltage to the motor, passed as a
     * {@link java.util.function.DoubleConsumer} to the {@link PIDCommand}. The
     * output will be set to zero if the RPM setpoint is zero, and will have the
     * {@link SimpleMotorFeedforward} calculqations applied to it when not.
     * 
     * @param pidOutput - the value passed from the {@link PIDCommand} to be
     *                  consumed
     */
    public void useOutputVolts(double pidOutput) {
        if (m_pid_bottom.getSetpoint() != 0) {
            double feedOutput = m_feedForward_bottom.calculate(m_pid_bottom.getSetpoint());
            m_talon_bottom.setVoltage(
                    pidOutput + feedOutput);
        } else {
            m_talon_bottom.setVoltage(0);
        }
    }

    /**
     * @return The velocity reading from the motor, in the units of rotations per
     *         second
     */
    public double getMotorRPM() {
        return m_talon_bottom.getVelocity().getValueAsDouble() * 60;
    }

    /**
     * @param setpoint - the nonnegative desired velocity in rotations per second of
     *                 the flywheels.
     * 
     */
    public void setSetpointRPM(double setpoint) {
        if (setpoint >= 0)
            m_setpoint = setpoint;
    }

    /**
     * Sets the velocity setpoint to zero rpm, as defined in {@code setSetpoint()}
     */
    public void stop() {
        setSetpointRPM(0);
    }

    /**
     * @return whether or not the flywheels are within 100 rpm of the desired velocity, thus
     *         if they are "spun up".
     */
    public boolean isSpunUp() {
        return Math.abs(m_pid_bottom.getPositionError()) < 100;
    }

    /**
     * @return a new {@link PIDCommand}, constructed with the subsystem's
     *         {@link PIDController}, rpm readings, desired velocity, and
     *         {@code useOutput()} method, also with the subsystem itself.
     */
    public PIDCommand shootPIDCommand() {
        return new PIDCommand(m_pid_bottom, () -> getMotorRPM(), () -> m_setpoint,
                a -> useOutputVolts(a), this);
    }
}
