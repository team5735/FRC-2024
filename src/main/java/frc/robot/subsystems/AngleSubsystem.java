package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AngleConstants;
import frc.robot.constants.Constants;

public class AngleSubsystem extends SubsystemBase{
    private PIDController m_pid;
    // private PIDController m_pid_left;
    private ArmFeedforward m_feedForward;
    // private PIDController m_feedForward_left;
    private boolean enabled = true;

    private final CANSparkMax m_sparkMax_right = new CANSparkMax(Constants.ANGLE_MOTOR_RIGHT_ID, MotorType.kBrushless);
    private final CANSparkMax m_sparkMax_left = new CANSparkMax(Constants.ANGLE_MOTOR_LEFT_ID, MotorType.kBrushless);

    private final RelativeEncoder m_encoder_right = m_sparkMax_right.getEncoder();


    public AngleSubsystem() {
        m_sparkMax_left.setInverted(true);
        m_sparkMax_right.setInverted(true);


        m_sparkMax_left.follow(m_sparkMax_right, true);


        m_pid = new PIDController(0, 0, 0);
        m_feedForward = new ArmFeedforward(0, 0, 0);

        updateProportions();

        double startAngle = AngleConstants.ANGLE_START_POS_ROT*AngleConstants.ANGLE_OUTPUT_TO_MOTOR_RATIO;

        m_encoder_right.setPosition(startAngle);

        m_pid.setSetpoint(AngleConstants.ANGLE_START_POS_DEG);
        // This is the actual value we are working with, when doing feedforward, we need to offset so that 0rad is parallel to base :)
    }

    @Override
    public void periodic() {
        updateProportions();

        useOutput();
        
        SmartDashboard.putNumber("anglePos", getMeasurement());
        SmartDashboard.putNumber("angleCurrentSetpoint", m_pid.getSetpoint());
    }

    public double getMeasurement() {
        return AngleConstants.convertRotationsToDegrees(m_encoder_right.getPosition() * AngleConstants.ANGLE_MOTOR_TO_OUTPUT_RATIO);
    }

    public void useOutput(){
        if(enabled){
            if(getMeasurement() < AngleConstants.ANGLE_LOWEST_DEG && m_pid.getSetpoint() < AngleConstants.ANGLE_HIGHEST_DEG)
                m_pid.setSetpoint(m_pid.getSetpoint() + 1);
            
            if(getMeasurement() > AngleConstants.ANGLE_HIGHEST_DEG && m_pid.getSetpoint() > AngleConstants.ANGLE_LOWEST_DEG)
                m_pid.setSetpoint(m_pid.getSetpoint() - 1);
    
            double rightOutput = m_pid.calculate(getMeasurement());
            m_sparkMax_right.setVoltage(rightOutput + m_feedForward.calculate(
                Math.toRadians(getMeasurement() - 90), rightOutput
            ));
        } else {
            m_sparkMax_right.setVoltage(0);
        }
    }

    public void updateProportions(){
        double rkp = SmartDashboard.getNumber("angleKP", AngleConstants.ANGLE_KP);
        double rki = SmartDashboard.getNumber("angleKI", AngleConstants.ANGLE_KI);
        double rkd = SmartDashboard.getNumber("angleKD", AngleConstants.ANGLE_KD);

        double rks = SmartDashboard.getNumber("angleKS", AngleConstants.ANGLE_KS);
        double rkg = SmartDashboard.getNumber("angleKG", AngleConstants.ANGLE_KG);
        double rkv = SmartDashboard.getNumber("angleKV", AngleConstants.ANGLE_KV);

        m_feedForward = new ArmFeedforward(rks, rkg, rkv);
        m_pid.setPID(rkp, rki, rkd);
    }

    public void pidReset() {
        m_pid.reset();
    }

    public void setSetpoint(double angle){
        m_pid.setSetpoint(angle);
    }

    public void releaseBrakes(){
        m_sparkMax_right.setIdleMode(IdleMode.kCoast);
        enabled = false;
    }

    public void engageBrakes(){
        m_sparkMax_right.setIdleMode(IdleMode.kBrake);
        enabled = true;
    }
}