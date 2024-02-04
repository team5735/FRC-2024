package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax_right = new CANSparkMax(Constants.CLIMBER_RIGHT_ID, MotorType.kBrushless);
    private final CANSparkMax m_sparkMax_left = new CANSparkMax(Constants.CLIMBER_LEFT_ID, MotorType.kBrushless);

    // private final PIDController m_speedController = new PIDController(1, 0, 0);
    // private final PIDController m_speedController = new PIDController(1, 0, 0);

    public void rightUp() {
        double rightVoltage =
            SmartDashboard.getNumber("climbRightUpVoltage", Constants.CLIMBER_RIGHT_UP_VOLTS);
        
        m_sparkMax_right.setVoltage(rightVoltage);

        SmartDashboard.putNumber("climbRightOutput", m_sparkMax_right.getOutputCurrent());
         SmartDashboard.putNumber("rightTrigger", 1);
    }

    public void leftUp(){
        double leftVoltage =
            SmartDashboard.getNumber("climbLeftUpVoltage", Constants.CLIMBER_LEFT_UP_VOLTS);

        m_sparkMax_left.setVoltage(leftVoltage);

        SmartDashboard.putNumber("climbLeftOutput", m_sparkMax_left.getOutputCurrent());
        SmartDashboard.putNumber("leftTrigger", 1);
    }

    public void rightDown() {
        double rightVoltage =
            SmartDashboard.getNumber("climbRightDownVoltage", Constants.CLIMBER_RIGHT_DOWN_VOLTS);
        
        m_sparkMax_right.setVoltage(-rightVoltage);

        SmartDashboard.putNumber("climbRightOutput", m_sparkMax_right.getOutputCurrent());
        SmartDashboard.putNumber("rightTrigger", -1);
    }

    public void leftDown(){
        double leftVoltage =
            SmartDashboard.getNumber("climbLeftDownVoltage", Constants.CLIMBER_LEFT_DOWN_VOLTS);
        
        m_sparkMax_left.setVoltage(-leftVoltage);

        SmartDashboard.putNumber("climbLeftOutput", m_sparkMax_left.getOutputCurrent());
        SmartDashboard.putNumber("leftTrigger", -1);
    }

    public void stopRight() {
        m_sparkMax_right.setIdleMode(IdleMode.kBrake);
        m_sparkMax_right.setVoltage(0);
         SmartDashboard.putNumber("rightTrigger", 0);
    }

    public void stopLeft(){
        m_sparkMax_left.setIdleMode(IdleMode.kBrake);
        m_sparkMax_left.setVoltage(0);
        SmartDashboard.putNumber("leftTrigger", 0);
    }
}
