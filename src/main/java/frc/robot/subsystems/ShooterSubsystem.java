package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private PIDController m_pid_top, m_pid_bottom;
    private SimpleMotorFeedforward m_feedForward_top, m_feedForward_bottom;

    private final TalonFX m_talon_top = new TalonFX(Constants.SHOOTER_MOTOR_TOP_ID);
    private final TalonFX m_talon_bottom = new TalonFX(Constants.SHOOTER_MOTOR_BOTTOM_ID);


    public ShooterSubsystem(){
        // m_talon_top.setInverted(true);
        // m_talon_bottom.setInverted(true);


        m_pid_top = new PIDController(0, 0, 0);
        m_feedForward_top = new SimpleMotorFeedforward(0, 0);

        m_pid_bottom = new PIDController(0, 0, 0);
        m_feedForward_bottom = new SimpleMotorFeedforward(0, 0);

        updateProportions();
    }

    // changes PID & FeedForward values based on the NetworkTables
    public void updateProportions(){
        double tkp = SmartDashboard.getNumber("shootTopKP", ShooterConstants.SHOOTER_TOP_KP);
        double tki = SmartDashboard.getNumber("shootTopKI", ShooterConstants.SHOOTER_TOP_KI);
        double tkd = SmartDashboard.getNumber("shootTopKD", ShooterConstants.SHOOTER_TOP_KD);

        double tks = SmartDashboard.getNumber("shootTopKS", ShooterConstants.SHOOTER_TOP_KS);
        double tkv = SmartDashboard.getNumber("shootTopKV", ShooterConstants.SHOOTER_TOP_KV);

        m_pid_top.setPID(tkp, tki, tkd);
        m_feedForward_top = new SimpleMotorFeedforward(tks, tkv);

        double bkp = SmartDashboard.getNumber("shootBottomKP", ShooterConstants.SHOOTER_BOTTOM_KP);
        double bki = SmartDashboard.getNumber("shootBottomKI", ShooterConstants.SHOOTER_BOTTOM_KI);
        double bkd = SmartDashboard.getNumber("shootBottomKD", ShooterConstants.SHOOTER_BOTTOM_KD);

        double bks = SmartDashboard.getNumber("shootBottomKS", ShooterConstants.SHOOTER_BOTTOM_KS);
        double bkv = SmartDashboard.getNumber("shootBottomKV", ShooterConstants.SHOOTER_BOTTOM_KV);

        m_pid_bottom.setPID(bkp, bki, bkd);
        m_feedForward_bottom = new SimpleMotorFeedforward(bks, bkv);
    }

    
    @Override
    public void periodic(){
        updateProportions();
        
        SmartDashboard.putNumber("shootTopOutput", Math.abs(getTopMeasurement()));
        SmartDashboard.putNumber("shootBottomOutput", Math.abs(getBottomMeasurement()));
    }

    public void useOutput(){
        if(m_pid_top.getSetpoint() != 0 || m_pid_bottom.getSetpoint() != 0 ){
            m_talon_top.setVoltage(
                m_pid_top.calculate(getTopMeasurement()) + m_feedForward_top.calculate(m_pid_top.getSetpoint())
            );
            m_talon_bottom.setVoltage(
                m_pid_bottom.calculate(getBottomMeasurement()) + m_feedForward_bottom.calculate(m_pid_bottom.getSetpoint())
            );
        } else {
            m_talon_top.setVoltage(0);
            m_talon_bottom.setVoltage(0);
        }
    }

    public double getTopMeasurement(){
        return m_talon_top.getVelocity().getValueAsDouble()*60;
    }

    public double getBottomMeasurement(){
        return m_talon_bottom.getVelocity().getValueAsDouble()*60;
    }

    
    public void start() {

        System.out.println("shootersubsystem");

        double topRPM =
            SmartDashboard.getNumber("shootTopRPM", ShooterConstants.SHOOTER_TOP_RPM);
        double bottomRPM =
            SmartDashboard.getNumber("shootBottomRPM", ShooterConstants.SHOOTER_BOTTOM_RPM);

        m_pid_top.setSetpoint(topRPM);
        m_pid_bottom.setSetpoint(bottomRPM);
    }

    public void stop() {
        m_pid_top.setSetpoint(0);
        m_pid_bottom.setSetpoint(0);
    }
}