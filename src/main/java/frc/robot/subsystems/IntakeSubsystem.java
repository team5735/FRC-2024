package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax_pull = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    public IntakeSubsystem() {
        m_sparkMax_pull.setInverted(true);
    }

    public void pull() {
        double pullSpeed = IntakeConstants.INTAKE_PULL_VOLTS;

        m_sparkMax_pull.setVoltage(pullSpeed);
    }

    public void push() {
        double pushSpeed = IntakeConstants.INTAKE_PUSH_VOLTS;

        m_sparkMax_pull.setVoltage(-pushSpeed);
    }

    public void stop() {
        m_sparkMax_pull.setVoltage(0);
    }

    public Command getPullStop() {
        return startEnd(() -> pull(), () -> stop());
    }

    public Command getPushStop() {
        return runOnce(() -> push());
    }

    public Command getStop() {
        return runOnce(() -> stop());
    }
}
