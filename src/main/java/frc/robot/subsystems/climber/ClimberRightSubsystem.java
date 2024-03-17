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

public class ClimberRightSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax_right = new CANSparkMax(Constants.CLIMBER_MOTOR_RIGHT_ID,
            MotorType.kBrushless);

    private final RelativeEncoder m_encoder_right = m_sparkMax_right.getEncoder();

    public ClimberRightSubsystem() {
        m_encoder_right.setPosition(0);

        m_sparkMax_right.setIdleMode(IdleMode.kBrake);
    }

    public void rightUp() {
        double rightVoltage = SmartDashboard.getNumber("climbRightUpVoltage", ClimberConstants.CLIMBER_RIGHT_UP_VOLTS);

        m_sparkMax_right.setVoltage(-rightVoltage);
    }

    public void rightDown() {
        double rightVoltage = SmartDashboard.getNumber("climbRightDownVoltage",
                ClimberConstants.CLIMBER_RIGHT_DOWN_VOLTS);

        m_sparkMax_right.setVoltage(rightVoltage);
    }

    public void stopRight() {
        m_sparkMax_right.setVoltage(0);
    }

    public void periodic() {
        SmartDashboard.putNumber("climbRightPos", m_encoder_right.getPosition());

        SmartDashboard.putNumber("climbRightOutput", m_sparkMax_right.getOutputCurrent());
    }

    public Command getUpStop() {
        return startEnd(() -> rightUp(), () -> stopRight());
    }

    public Command getDownStop() {
        return startEnd(() -> rightDown(), () -> stopRight());
    }

    public Command getStop() {
        return runOnce(() -> stopRight());
    }
}
