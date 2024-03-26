package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;

public class ClimberLeftSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax_left = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT_ID,
            MotorType.kBrushless);

    private final RelativeEncoder m_encoder_left = m_sparkMax_left.getEncoder();

    public ClimberLeftSubsystem() {
        m_encoder_left.setPosition(0);

        m_sparkMax_left.setIdleMode(IdleMode.kBrake);
    }

    public void leftUp() {
        double leftVoltage = ClimberConstants.CLIMBER_LEFT_UP_VOLTS;

        m_sparkMax_left.setVoltage(-leftVoltage);

    }

    public void leftDown() {
        double leftVoltage = ClimberConstants.CLIMBER_LEFT_DOWN_VOLTS;

        m_sparkMax_left.setVoltage(leftVoltage);
    }

    public void stopLeft() {
        m_sparkMax_left.setVoltage(0);
    }

    public void periodic() {
        SmartDashboard.putNumber("climbLeftPos", m_encoder_left.getPosition());

        SmartDashboard.putNumber("climbLeftOutput", m_sparkMax_left.getOutputCurrent());
    }

    public Command getUpStop() {
        return startEnd(() -> leftUp(), () -> stopLeft());
    }

    public Command getDownStop() {
        return startEnd(() -> leftDown(), () -> stopLeft());
    }

    public Command getStop() {
        return runOnce(() -> stopLeft());
    }
}
