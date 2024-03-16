package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax_pull = new CANSparkMax(Constants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput m_switch = new DigitalInput(Constants.FEEDER_BEAM_PIN);

    public FeederSubsystem() {
        m_sparkMax_pull.setInverted(true);
    }

    public void pull() {
        double pullSpeed = SmartDashboard.getNumber("feederPullVoltage", FeederConstants.FEEDER_PULL_VOLTS);

        m_sparkMax_pull.setVoltage(pullSpeed);
    }

    public void push() {
        double pushSpeed = SmartDashboard.getNumber("feederPushVoltage", FeederConstants.FEEDER_PUSH_VOLTS);

        m_sparkMax_pull.setVoltage(-pushSpeed);
    }

    public void stop() {
        m_sparkMax_pull.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("feederSwitchStatus", getSwitchStatus());
        SmartDashboard.putNumber("feederVoltage", m_sparkMax_pull.getBusVoltage());
    }

    public boolean getSwitchStatus() {
        return m_switch.get();
    }

    public Command getPull() {
        return startEnd(() -> pull(), () -> stop());
    }

    public Command getPush() {
        return startEnd(() -> push(), () -> stop());
    }

    public Command getStop() {
        return runOnce(() -> stop());
    }
}
