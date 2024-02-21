package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_talon_top = new TalonFX(Constants.SHOOTER_MOTOR_TOP_ID);
    private final TalonFX m_talon_bottom = new TalonFX(Constants.SHOOTER_MOTOR_BOTTOM_ID);

    // private final PIDController m_speedController = new PIDController(1, 0, 0);

    public ShooterSubsystem(){
        // m_talon_top.setNeutralMode(NeutralModeValue.Coast);
        // m_talon_bottom.setNeutralMode(NeutralModeValue.Coast);
    }

    public void drive() {
        double topVoltage =
            SmartDashboard.getNumber("shootTopVoltage", ShooterConstants.SHOOTER_TOP_VOLTS);
        double bottomVoltage =
            SmartDashboard.getNumber("shootBottomVoltage", ShooterConstants.SHOOTER_BOTTOM_VOLTS);


        m_talon_top.setVoltage(topVoltage);
        m_talon_bottom.setVoltage(bottomVoltage);

        SmartDashboard.putNumber("shootTopRPS", m_talon_top.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("shootBottomRPS", m_talon_bottom.getVelocity().getValueAsDouble());
    }

    public void periodic(){
        SmartDashboard.putBoolean("shootTest", true);
    }

    public void stop() {
        m_talon_top.setVoltage(0);
        m_talon_bottom.setVoltage(0);
    }
}
