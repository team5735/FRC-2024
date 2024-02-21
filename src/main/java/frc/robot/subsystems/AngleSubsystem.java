package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AngleConstants;
import frc.robot.constants.Constants;

public class AngleSubsystem extends SubsystemBase{
    /*
     * 
     * 
     * DO NOT USE THIS SUBSYSTEM ON THIS BRANCH!!!!!!!!
     * Use the PID-&-FeedForward branch for this subsytem, for now 
     * 
     * 
     * 
     */
    private ArmFeedforward m_feedForward;
    private PIDController m_pid;

    private TalonFX m_talon_right = new TalonFX(Constants.ANGLE_MOTOR_RIGHT_ID);
    private TalonFX m_talon_left = new TalonFX(Constants.ANGLE_MOTOR_LEFT_ID);

    public AngleSubsystem() {
        double kp = SmartDashboard.getNumber("angleKP", AngleConstants.ANGLE_KP);
        double ki = SmartDashboard.getNumber("angleKI", AngleConstants.ANGLE_KI);
        double kd = SmartDashboard.getNumber("angleKD", AngleConstants.ANGLE_KD);

        double ks = SmartDashboard.getNumber("angleKS", AngleConstants.ANGLE_KS);
        double kg = SmartDashboard.getNumber("angleKG", AngleConstants.ANGLE_KG);
        double kV = SmartDashboard.getNumber("angleKV", AngleConstants.ANGLE_KV);

        m_feedForward = new ArmFeedforward(ks, kg, kV);
        m_pid = new PIDController(kp, ki, kd);

        m_talon_left.setPosition(0); // Need to find the correct position on start, 0 should be parallel to robot
                                     // base
    }

    public void periodic() {
        double kp = SmartDashboard.getNumber("angleKP", AngleConstants.ANGLE_KP);
        double ki = SmartDashboard.getNumber("angleKI", AngleConstants.ANGLE_KI);
        double kd = SmartDashboard.getNumber("angleKD", AngleConstants.ANGLE_KD);

        double ks = SmartDashboard.getNumber("angleKS", AngleConstants.ANGLE_KS);
        double kg = SmartDashboard.getNumber("angleKG", AngleConstants.ANGLE_KG);
        double kV = SmartDashboard.getNumber("angleKV", AngleConstants.ANGLE_KV);

        m_feedForward = new ArmFeedforward(ks, kg, kV);
        m_pid.setPID(kp, ki, kd);

        useOutput(m_pid.calculate(getMeasurement()), m_pid.getSetpoint());

        SmartDashboard.putNumber("anglePos", getMeasurement());
    }

    public double getMeasurement() {
        return AngleConstants.convertTalonRotationsToDegrees(m_talon_left.getPosition().getValueAsDouble());
    }

    public void useOutput(double output, double setpoint) {
        m_talon_right.setVoltage(output + m_feedForward.calculate(setpoint, output));
        m_talon_left.setVoltage(output + m_feedForward.calculate(setpoint, output));
    }

    public void pidReset() {
        m_pid.reset();
    }

    public void setSetpoint(double angle){
        m_pid.setSetpoint(angle);
    }
}
