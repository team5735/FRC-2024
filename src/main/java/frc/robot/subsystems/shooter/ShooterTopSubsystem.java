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
 * This class represents a ShooterTopSubsystem. It utilizes a sequence of
 * compliant wheels attatched to an axle driven by an {@link TalonFX}. The Talon
 * is controlled via a {@link PIDController} and a
 * {@link SimpleMotorFeedforward} for accurate and precise speed management.
 * 
 * @author Jacoby (with help for Commands)
 */
public class ShooterTopSubsystem extends SubsystemBase {
    private PIDController m_pid_top;
    private SimpleMotorFeedforward m_feedForward_top;
    private double m_setpoint;

    private final TalonFX m_talon_top = new TalonFX(Constants.SHOOTER_MOTOR_TOP_ID);

    /**
     * Creates a new ShooterBottom Subsystem, setting the motor to coast mode and
     * initizalizing PID and FF values via {@code updateProportions()}
     */
    public ShooterTopSubsystem() {
        m_talon_top.setNeutralMode(NeutralModeValue.Coast);
        m_talon_top.setInverted(false);

        m_pid_top = new PIDController(0, 0, 0);
        m_feedForward_top = new SimpleMotorFeedforward(0, 0);

        updateProportions();
    }

    /**
     * Gets values from {@link ShooterConstants} for the {@link PIDController} and
     * the
     * {@link SimpleMotorFeedforward}. Then, {@code m_pid_top} and {@code m_feedForward_top}
     * are reconstructed based on the values acquired from {@link ShooterConstants}.
     *
     * <p>
     * Uses the following values:
     * <ul>
     * <li>{@code TOP_KP}
     * <li>{@code TOP_KI}
     * <li>{@code TOP_KD}
     * <li>{@code TOP_KS}
     * <li>{@code TOP_KV}
     * </ul>
     */
    public void updateProportions() {
        double tkp = ShooterConstants.TOP_KP;
        double tki = ShooterConstants.TOP_KI;
        double tkd = ShooterConstants.TOP_KD;

        double tks = ShooterConstants.TOP_KS;
        double tkv = ShooterConstants.TOP_KV;

        m_pid_top.setPID(tkp, tki, tkd);
        m_feedForward_top = new SimpleMotorFeedforward(tks, tkv);
    }

    /**
     * Periodic method called by the CommandScheduler, changes the PID and FF values
     * via {@code UpdateProportions()} and logs values to {@link SmartDashboard}.
     */
    @Override
    public void periodic() {
        // updateProportions();

        SmartDashboard.putNumber("shootTopOutput", Math.abs(getTopMeasurement()));
        SmartDashboard.putNumber("shootTopPIDError", Math.abs(m_pid_top.getPositionError()));
        SmartDashboard.putNumber("shootTopSetpoint", m_setpoint);
        SmartDashboard.putNumber("shootTopAmps", m_talon_top.getStatorCurrent().getValueAsDouble());
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
    public void useOutput(double pidOutput) {
        if (m_pid_top.getSetpoint() != 0) {
            double feedOutput = m_feedForward_top.calculate(m_pid_top.getSetpoint());
            m_talon_top.setVoltage(
                    pidOutput + feedOutput);
        } else {
            m_talon_top.setVoltage(0);
        }
    }

    /**
     * @return The velocity reading from the motor, in the units of rotations per
     *         second
     */
    public double getTopMeasurement() {
        return m_talon_top.getVelocity().getValueAsDouble() * 60;
    }

    /**
     * @param setpoint - the nonnegative desired velocity in rotations per second of
     *                 the flywheels.
     * 
     */
    public void setSetpoint(double setpoint) {
        if (setpoint >= 0)
            m_setpoint = setpoint;
    }

    /**
     * Sets the velocity setpoint to zero rpm, as defined in {@code setSetpoint()}
     */
    public void stop() {
        setSetpoint(0);
    }

    /**
     * @return whether or not the flywheels are within 100 rpm of the desired velocity, thus
     *         if they are "spun up".
     */
    public boolean isSpunUp() {
        return (Math.abs(m_pid_top.getPositionError()) < 100);
    }

    /**
     * @return a new {@link PIDCommand}, constructed with the subsystem's
     *         {@link PIDController}, rpm readings, desired velocity, and
     *         {@code useOutput()} method, also with the subsystem itself.
     */
    public PIDCommand shootPIDCommand() {
        return new PIDCommand(m_pid_top, () -> getTopMeasurement(), () -> m_setpoint, a -> useOutput(a),
                this);
    }
}
