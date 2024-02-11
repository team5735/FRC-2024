package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_talon_right = new TalonFX(Constants.SHOOTER_MOTOR_RIGHT_ID);
    private final TalonFX m_talon_left = new TalonFX(Constants.SHOOTER_MOTOR_LEFT_ID);

    // private final PIDController m_speedController = new PIDController(1, 0, 0);

    public ShooterSubsystem(){
        // m_talon_right.setInverted(true);
    }

    public void drive() {
        double rightVoltage =
            SmartDashboard.getNumber("shootRightVoltage", ShooterConstants.SHOOTER_RIGHT_VOLTS);
        double leftVoltage =
            SmartDashboard.getNumber("shootLeftVoltage", ShooterConstants.SHOOTER_LEFT_VOLTS);


        m_talon_right.setVoltage(rightVoltage);
        m_talon_left.setVoltage(-leftVoltage);

        SmartDashboard.putNumber("shootRightOutput", m_talon_right.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("shootLeftOutput", m_talon_left.getVelocity().getValueAsDouble());
    }

    public void stop() {
        m_talon_right.setVoltage(0);
        m_talon_left.setVoltage(0);
    }
}
