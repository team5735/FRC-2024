package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_talon_pull = new TalonFX(Constants.INTAKE_MOTOR_ID);

    public IntakeSubsystem() {
        m_talon_pull.setInverted(true);
    }

    public void pull() {
        double pullSpeed = SmartDashboard.getNumber("intakePullVoltage", IntakeConstants.INTAKE_PULL_VOLTS);

        m_talon_pull.setVoltage(pullSpeed);
    }

    public void stop() {
        m_talon_pull.setVoltage(0);
    }

}
