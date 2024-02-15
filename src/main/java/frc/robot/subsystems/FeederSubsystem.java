package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax_pull = new CANSparkMax(Constants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput m_switch = new DigitalInput(Constants.FEEDER_SWITCH_CHANNEL);

    public FeederSubsystem() {
        // m_sparkMax_pull.setInverted(true);
    }

    public void pull() {
        double pullSpeed = SmartDashboard.getNumber("feederPullVoltage", FeederConstants.FEEDER_PULL_VOLTS);

        m_sparkMax_pull.setVoltage(pullSpeed);
    }

    public void stop() {
        m_sparkMax_pull.setVoltage(0);
    }

    public void periodic(){
        SmartDashboard.putBoolean("feederSwitchStatus", getSwitchStatus());
    }

    public boolean getSwitchStatus() {
        return m_switch.get();
    }

}
