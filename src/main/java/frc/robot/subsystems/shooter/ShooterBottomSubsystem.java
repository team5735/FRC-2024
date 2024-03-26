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

public class ShooterBottomSubsystem extends SubsystemBase {
    private PIDController m_pid_bottom;
    private SimpleMotorFeedforward m_feedForward_bottom;
    private double m_setpoint;

    private final TalonFX m_talon_bottom = new TalonFX(Constants.SHOOTER_MOTOR_BOTTOM_ID);

    public ShooterBottomSubsystem() {
        m_talon_bottom.setNeutralMode(NeutralModeValue.Coast);

        m_pid_bottom = new PIDController(0, 0, 0);
        m_feedForward_bottom = new SimpleMotorFeedforward(0, 0);

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
     * <li>shootBottomKP
     * <li>shootBottomKI
     * <li>shootBottomKD
     * <li>shootBottomKS
     * <li>shootBottomKV
     * </ul>
     */
    public void updateProportions() {
        double bkp = ShooterConstants.SHOOTER_BOTTOM_KP;
        double bki = ShooterConstants.SHOOTER_BOTTOM_KI;
        double bkd = ShooterConstants.SHOOTER_BOTTOM_KD;

        double bks = ShooterConstants.SHOOTER_BOTTOM_KS;
        double bkv = ShooterConstants.SHOOTER_BOTTOM_KV;

        m_pid_bottom.setPID(bkp, bki, bkd);
        m_feedForward_bottom = new SimpleMotorFeedforward(bks, bkv);
    }

    @Override
    public void periodic() {
        updateProportions();

        SmartDashboard.putNumber("shootBottomOutput", Math.abs(getBottomMeasurement()));
        SmartDashboard.putNumber("shootBottomPIDError", Math.abs(m_pid_bottom.getPositionError()));
        SmartDashboard.putNumber("shootBottomSetpoint", m_setpoint);
        SmartDashboard.putNumber("shootTopAmps", m_talon_bottom.getStatorCurrent().getValueAsDouble());

    }

    public void useOutput(double pidOutput) {
        if (m_pid_bottom.getSetpoint() != 0) {
            double feedOutput = m_feedForward_bottom.calculate(m_pid_bottom.getSetpoint());
            m_talon_bottom.setVoltage(
                    pidOutput + feedOutput);
        } else {
            m_talon_bottom.setVoltage(0);
        }
    }

    public double getBottomMeasurement() {
        return m_talon_bottom.getVelocity().getValueAsDouble() * 60;
    }

    public void setSetpoint(double setpoint) {
        if (setpoint >= 0)
            m_setpoint = setpoint;
    }

    public void stop() {
        setSetpoint(0);
    }

    public boolean isSpunUp() {
        return Math.abs(m_pid_bottom.getPositionError()) < 100;
    }

    public PIDCommand shootPIDCommand() {
        return new PIDCommand(m_pid_bottom, () -> getBottomMeasurement(), () -> m_setpoint,
                a -> useOutput(a), this);
    }
}
