package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.angle.AngleCommandSetAngle;
import frc.robot.constants.AngleConstants;
import frc.robot.constants.Constants;

public class AngleSubsystem extends SubsystemBase {
    private PIDController m_pid;
    private ArmFeedforward m_feedForward;
    private boolean enabled = true;
    private double startPosition = 0;
    private double m_setpoint, m_activeOutput;

    private final CANSparkMax m_sparkMax_right = new CANSparkMax(
            Constants.ANGLE_MOTOR_RIGHT_ID, MotorType.kBrushless);
    private final CANSparkMax m_sparkMax_left = new CANSparkMax(
            Constants.ANGLE_MOTOR_LEFT_ID, MotorType.kBrushless);

    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(Constants.ANGLE_ENCODER_PIN);

    public AngleSubsystem() {
        m_sparkMax_left.setInverted(true);
        m_sparkMax_right.setInverted(true);

        m_sparkMax_left.follow(m_sparkMax_right, true);

        m_pid = new PIDController(0, 0, 0);
        m_feedForward = new ArmFeedforward(0, 0, 0);

        m_pid.setIZone(1);

        updateProportions();

        m_encoder.setDistancePerRotation(1);

        setSetpoint(AngleConstants.ANGLE_START_POS_DEG);
    }

    // overriden method called every 20ms, calls updateProportions
    // as well as updating the NetworkTables for certain readings
    @Override
    public void periodic() {
        updateProportions();

        SmartDashboard.putNumber("anglePos", getMeasurement());
        SmartDashboard.putNumber("angleCurrentSetpoint", m_setpoint);
        SmartDashboard.putNumber("angleLeftAmps", m_sparkMax_left.getOutputCurrent());
        SmartDashboard.putNumber("angleRightAmps", m_sparkMax_right.getOutputCurrent());
        SmartDashboard.putNumber("anglePIDError", Math.abs(m_pid.getPositionError()));
        SmartDashboard.putNumber("anglePIDOutput", m_activeOutput);
    }

    // reads the motor's position and multiplies it by the constant ratio to
    // determine the arm's position
    public double getMeasurement() {
        // return AngleConstants.convertRotationsToDegrees(m_encoder.getDistance());
        if (startPosition == 0) {
            startPosition = m_encoder.getDistance();
        }
        double currentAngleDegrees = AngleConstants.convertRotationsToDegrees(
                m_encoder.getDistance() - startPosition + AngleConstants.ANGLE_START_POS_ROT);

        return 0.1 * Math.round(currentAngleDegrees * 10);
    }

    // sets the motor voltage to the PID & FeedForward calculations
    public void useOutput(double pidOutput) {
        if (enabled) {
            if (getMeasurement() < AngleConstants.ANGLE_LOWEST_DEG
                    && m_setpoint < AngleConstants.ANGLE_HIGHEST_DEG)
                setSetpoint(m_pid.getSetpoint() + 1);

            if (getMeasurement() > AngleConstants.ANGLE_HIGHEST_DEG
                    && m_setpoint > AngleConstants.ANGLE_LOWEST_DEG)
                setSetpoint(m_pid.getSetpoint() - 1);

            double feedOutput = (!isAtBase())
                    ? m_feedForward.calculate(Math.toRadians(getMeasurement()), pidOutput)
                    : 0;
            double volts = pidOutput + feedOutput;
            m_sparkMax_right.setVoltage(volts);
            SmartDashboard.putNumber("angleHypotheticalOutput", volts);
        } else {
            m_sparkMax_right.setVoltage(0);
        }
        m_activeOutput = pidOutput;
    }

    /**
     * Gets values from {@link SmartDashboard} for the {@link PIDController} and the
     * {@link SimpleMotorFeedforward}. Then, m_pid_bottom and m_feedForward_bottom
     * are reconstructed based on the values acquired from {@link SmartDashboard.}
     *
     * <p>
     * Uses the following values:
     * <ul>
     * <li>angleKP
     * <li>angleKI
     * <li>angleKD
     * <li>angleKS
     * <li>angleKG
     * <li>angleKV
     * </ul>
     */
    public void updateProportions() {
        double kp = SmartDashboard.getNumber("angleKP", AngleConstants.ANGLE_KP);
        double ki = SmartDashboard.getNumber("angleKI", AngleConstants.ANGLE_KI);
        double kd = SmartDashboard.getNumber("angleKD", AngleConstants.ANGLE_KD);

        double ks = SmartDashboard.getNumber("angleKS", AngleConstants.ANGLE_KS);
        double kg = SmartDashboard.getNumber("angleKG", AngleConstants.ANGLE_KG);
        double kv = SmartDashboard.getNumber("angleKV", AngleConstants.ANGLE_KV);

        m_feedForward = new ArmFeedforward(ks, kg, kv);
        m_pid.setPID(kp, ki, kd);
    }

    public void pidReset() {
        m_pid.reset();
    }

    public void setSetpoint(double angle) {
        if (angle > AngleConstants.ANGLE_LOWEST_DEG && angle < AngleConstants.ANGLE_HIGHEST_DEG)
            m_setpoint = angle;
    }

    // disables the PID & FeedForward, sets the motors to loose, and holds the
    // voltage at 0
    public void releaseBrakes() {
        m_sparkMax_right.setIdleMode(IdleMode.kCoast);
        enabled = false;
    }

    // undoes the previous method (re-enables controllers & voltage)
    public void engageBrakes() {
        m_sparkMax_right.setIdleMode(IdleMode.kBrake);
        enabled = true;
    }

    public PIDCommand anglePIDCommand(AngleSubsystem s) {
        return new PIDCommand(m_pid, () -> getMeasurement(), () -> {
            return m_setpoint;
        }, a -> useOutput(a), s);
    }

    public boolean isAtSetpoint() {
        return Math.abs(m_pid.getPositionError()) < 5;
    }

    public boolean isAtBase() {
        return Math.abs(getMeasurement() - AngleConstants.ANGLE_START_POS_DEG) < 2;
    }

    public boolean isAtPosition(double setPos) {
        return Math.abs(getMeasurement() - setPos) < 5;
    }

    public Command angleToBase() {
        // return new SequentialCommandGroup(
        // new AngleCommandSetAngle(this, AngleConstants.ANGLE_START_POS_DEG - 15),
        return new AngleCommandSetAngle(this, AngleConstants.ANGLE_START_POS_DEG);
        // );
    }

    public Command angleToMax() {
        return new AngleCommandSetAngle(this, AngleConstants.ANGLE_LOWEST_DEG + 10);
    }

    public Command angleToFarthestSpeaker() {
        return new AngleCommandSetAngle(this, AngleConstants.ANGLE_FARTHEST_SHOOT_DEG);
    }

    public Command angleIncrease() {
        return new RepeatCommand(new AngleCommandSetAngle(this, m_setpoint - 10));
    }

    public Command angleDecrease() {
        return new RepeatCommand(new AngleCommandSetAngle(this, m_setpoint + 10));
    }
}
