package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax_right = new CANSparkMax(Constants.CLIMBER_MOTOR_RIGHT_ID,
            MotorType.kBrushless);
    private final CANSparkMax m_sparkMax_left = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT_ID,
            MotorType.kBrushless);

    private final RelativeEncoder m_encoder_right = m_sparkMax_right.getEncoder();


    public void rightUp() {
        double rightVoltage = SmartDashboard.getNumber("climbRightUpVoltage", ClimberConstants.CLIMBER_RIGHT_UP_VOLTS);

        m_sparkMax_right.setVoltage(rightVoltage);

        SmartDashboard.putNumber("climbRightOutput", m_sparkMax_right.getOutputCurrent());
    }

    public void leftUp() {
        double leftVoltage = SmartDashboard.getNumber("climbLeftUpVoltage", ClimberConstants.CLIMBER_LEFT_UP_VOLTS);

        m_sparkMax_left.setVoltage(leftVoltage);

        SmartDashboard.putNumber("climbLeftOutput", m_sparkMax_left.getOutputCurrent());
    }

    public void rightDown() {
        double rightVoltage = SmartDashboard.getNumber("climbRightDownVoltage",
                ClimberConstants.CLIMBER_RIGHT_DOWN_VOLTS);

        m_sparkMax_right.setVoltage(-rightVoltage);

        SmartDashboard.putNumber("climbRightOutput", m_sparkMax_right.getOutputCurrent());
    }

    public void leftDown() {
        double leftVoltage = SmartDashboard.getNumber("climbLeftDownVoltage", ClimberConstants.CLIMBER_LEFT_DOWN_VOLTS);

        m_sparkMax_left.setVoltage(-leftVoltage);

        SmartDashboard.putNumber("climbLeftOutput", m_sparkMax_left.getOutputCurrent());
    }

    public void stopRight() {
        m_sparkMax_right.setIdleMode(IdleMode.kBrake);
        m_sparkMax_right.setVoltage(0);
    }

    public void stopLeft() {
        m_sparkMax_left.setIdleMode(IdleMode.kBrake);
        m_sparkMax_left.setVoltage(0);
    }
}
