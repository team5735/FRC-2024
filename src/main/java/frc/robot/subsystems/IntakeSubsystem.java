package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax_pull = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput m_switch = new DigitalInput(Constants.INTAKE_BEAM_PIN);

    public IntakeSubsystem() {
        m_sparkMax_pull.setInverted(true);
    }

    public void pull() {
        // double pullSpeed = SmartDashboard.getNumber("intakePullVoltage",
        // IntakeConstants.INTAKE_PULL_VOLTS);
        double pullSpeed = IntakeConstants.INTAKE_PULL_VOLTS;

        m_sparkMax_pull.setVoltage(pullSpeed);
    }

    public void push() {
        // double pushSpeed = SmartDashboard.getNumber("intakePushVoltage",
        // IntakeConstants.INTAKE_PUSH_VOLTS);
        double pushSpeed = IntakeConstants.INTAKE_PUSH_VOLTS;

        m_sparkMax_pull.setVoltage(-pushSpeed);
    }

    public void stop() {
        m_sparkMax_pull.setVoltage(0);
    }

    /**
     * Logs values to {@link SmartDashboard}.
     * 
     * <p>
     * {@code "intakeSwitchStatus"} - The boolean state of the beam break
     *
     * <p>
     * Also checks the intake switch.
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intakeSwitchStatus", getSwitchStatus());
    }

    /**
     * @return the boolean state of the {@link DigitalInput} beam break of this
     *         subsystem
     */
    public boolean getSwitchStatus() {
        return m_switch.get();
    }


    public Command getPullStop() {
        return startEnd(() -> pull(), () -> stop());
    }

    public Command getPushStop() {
        return startEnd(() -> push(), () -> stop());
    }

    public Command getStop() {
        return runOnce(() -> stop());
    }

        public Trigger beamBreakEngaged() {
        return new Trigger(() -> !getSwitchStatus());
    }
}
