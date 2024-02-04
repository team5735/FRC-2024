package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_talon_right = new TalonFX(Constants.SHOOTER_RIGHT_ID);
    private final TalonFX m_talon_left = new TalonFX(Constants.SHOOTER_LEFT_ID);

    // private final PIDController m_speedController = new PIDController(1, 0, 0);

    public void drive() {
        double rightVoltage =
            SmartDashboard.getNumber("rightShootVoltage", Constants.SHOOTER_SPEED);
        double leftVoltage =
            SmartDashboard.getNumber("leftShootVoltage", Constants.SHOOTER_SPEED);

        m_talon_right.setInverted(true);

        m_talon_right.setVoltage(rightVoltage);
        m_talon_left.setVoltage(leftVoltage);

        SmartDashboard.putNumber("rightShootSpeed", m_talon_right.get());
        SmartDashboard.putNumber("leftShootSpeed", m_talon_left.get());
    }

    public void stop() {
        m_talon_right.setVoltage(0);
        m_talon_left.setVoltage(0);
    }
}
