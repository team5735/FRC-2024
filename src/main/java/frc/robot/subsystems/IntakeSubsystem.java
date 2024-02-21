package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private PIDController m_pid;
    private SimpleMotorFeedforward m_feedForward;

    private final CANSparkMax m_sparkMax_pull = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder m_encoder_pull = m_sparkMax_pull.getEncoder();

    public IntakeSubsystem() {
        m_sparkMax_pull.setInverted(true);

        m_pid = new PIDController(0, 0, 0);

        m_feedForward = new SimpleMotorFeedforward(0, 0);

        updateProportions();
    }

    public void updateProportions(){
        double kp = SmartDashboard.getNumber("intakeKP", IntakeConstants.INTAKE_KP);
        double ki = SmartDashboard.getNumber("intakeKI", IntakeConstants.INTAKE_KI);
        double kd = SmartDashboard.getNumber("intakeKD", IntakeConstants.INTAKE_KD);

        double ks = SmartDashboard.getNumber("intakeKS", IntakeConstants.INTAKE_KS);
        double kv = SmartDashboard.getNumber("intakeKV", IntakeConstants.INTAKE_KV);

        m_pid.setPID(kp, ki, kd);
        m_feedForward = new SimpleMotorFeedforward(ks, kv);
    }

    public void periodic(){
        updateProportions();
        
        SmartDashboard.putNumber("intakeOutput", getMeasurement());
    }

    public void useOutput(){
        if(m_pid.getSetpoint() != 0){
            m_sparkMax_pull.setVoltage(m_pid.calculate(getMeasurement()) + m_feedForward.calculate(m_pid.getSetpoint()));
        } else {
            m_sparkMax_pull.setVoltage(0);
        }
    }

    public double getMeasurement(){
        return m_encoder_pull.getVelocity();
    }

    public void pull() {
        double pullSpeed = SmartDashboard.getNumber("intakePullRPM", IntakeConstants.INTAKE_PULL_RPM);

        m_pid.setSetpoint(pullSpeed);
    }

    public void push() {
        double pushSpeed = SmartDashboard.getNumber("intakePushRPM", IntakeConstants.INTAKE_PUSH_RPM);

        m_pid.setSetpoint(pushSpeed);
    }

    public void stop() {
        m_pid.setSetpoint(0);
    }

}
