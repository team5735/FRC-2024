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

public class ShooterTopSubsystem extends SubsystemBase {
    private PIDController m_pid_top;
    private SimpleMotorFeedforward m_feedForward_top;
    private double m_setpoint;

    private final TalonFX m_talon_top = new TalonFX(Constants.SHOOTER_MOTOR_TOP_ID);

    public ShooterTopSubsystem() {
        m_talon_top.setNeutralMode(NeutralModeValue.Coast);
        m_talon_top.setInverted(false);

        m_pid_top = new PIDController(0, 0, 0);
        m_feedForward_top = new SimpleMotorFeedforward(0, 0);

        updateProportions();
    }

    /**
     * Gets values from {@link SmartDashboard} for the {@link PIDController} and the
     * {@link SimpleMotorFeedforward}. Then, m_pid_bottom and m_feedForward_bottom
     * are reconstructed based on the values acquired from {@link SmartDashboard.}
     *
     * <p>
     * Uses the following values:
     * <ul>
     * <li>shootTopKP
     * <li>shootTopKI
     * <li>shootTopKD
     * <li>shootTopKS
     * <li>shootTopKV
     * </ul>
     */
    public void updateProportions() {
        double tkp = ShooterConstants.SHOOTER_TOP_KP;
        double tki = ShooterConstants.SHOOTER_TOP_KI;
        double tkd = ShooterConstants.SHOOTER_TOP_KD;

        double tks = ShooterConstants.SHOOTER_TOP_KS;
        double tkv = ShooterConstants.SHOOTER_TOP_KV;

        m_pid_top.setPID(tkp, tki, tkd);
        m_feedForward_top = new SimpleMotorFeedforward(tks, tkv);
    }

    @Override
    public void periodic() {
        // updateProportions();

        SmartDashboard.putNumber("shootTopOutput", Math.abs(getTopMeasurement()));
        SmartDashboard.putNumber("shootTopPIDError", Math.abs(m_pid_top.getPositionError()));
        SmartDashboard.putNumber("shootTopSetpoint", m_setpoint);
        SmartDashboard.putNumber("shootTopAmps", m_talon_top.getStatorCurrent().getValueAsDouble());
    }

    public void useOutput(double pidOutput) {
        if (m_pid_top.getSetpoint() != 0) {
            double feedOutput = m_feedForward_top.calculate(m_pid_top.getSetpoint());
            m_talon_top.setVoltage(
                    pidOutput + feedOutput);
        } else {
            m_talon_top.setVoltage(0);
        }
    }

    public double getTopMeasurement() {
        return m_talon_top.getVelocity().getValueAsDouble() * 60;
    }

    public void setSetpoint(double setpoint) {
        if (setpoint >= 0)
            m_setpoint = setpoint;
    }

    public void stop() {
        setSetpoint(0);
    }

    public boolean isSpunUp() {
        return (Math.abs(m_pid_top.getPositionError()) < 100);
    }

    public PIDCommand shootPIDCommand() {
        return new PIDCommand(m_pid_top, () -> getTopMeasurement(), () -> m_setpoint, a -> useOutput(a),
                this);
    }
}
