package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase

{
    // private final CANSparkMax m_sparkMax_pull = new CANSparkMax(Constants.INTAKE_PULL_ID,
    //         com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    private final TalonFX m_talon_pull = new TalonFX(Constants.INTAKE_PULL_ID);

    public IntakeSubsystem() {
        m_talon_pull.setInverted(true);
    }

    public void pull() {
        double pullSpeed = SmartDashboard.getNumber("intakePullSpeed", Constants.INTAKE_PULL_SPEED);

        m_talon_pull.setVoltage(pullSpeed);

    }

    public void push() {
        double pushSpeed = SmartDashboard.getNumber("intakePushSpeed", Constants.INTAKE_PUSH_SPEED);

        m_talon_pull.setVoltage(-pushSpeed);
    }

    public void stop() {
        m_talon_pull.setVoltage(0);
    }

}