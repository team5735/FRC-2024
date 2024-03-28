package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FactoryCommands;
import frc.robot.constants.AngleConstants;
import frc.robot.constants.Constants;

/**
 * This class represents the angle changer subsystem. It uses a PID to try and
 * maintain a specific angle while using a feed forward to overcome gravity,
 * except for when the angle changer is at rest. The setpoint can be set with
 * setSetpoint.
 *
 * @author Jacoby (with formatting and Command help from others)
 */
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

    /**
     * Creates a new AngleSubsystem. Inverts both motors, and sets the left motor to
     * follow the right. Also initializes the PID and feed forward using the
     * constants in AngleConstants and gives the PID an IZone of 1. Finally, the
     * setpoint is set to the resting position and the encoder is initialized.
     */
    public AngleSubsystem() {
        m_sparkMax_left.setInverted(true);
        m_sparkMax_right.setInverted(true);

        m_sparkMax_left.follow(m_sparkMax_right, true);

        m_pid = new PIDController(0, 0, 0);
        m_feedForward = new ArmFeedforward(0, 0, 0);

        updateProportions();

        m_encoder.setDistancePerRotation(1);

        setSetpoint(AngleConstants.BASE_POS_DEG);
    }

    /**
     * This implementation of periodic() simply puts a few numbers into
     * SmartDashboard for debugging purposes. The only reason we don't use an
     * {@link frc.robot.util.NTDoubleSection} here is because the code hasn't been
     * touched since that was implemented.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("anglePos", getMeasurement());
        SmartDashboard.putNumber("angleCurrentSetpoint", m_setpoint);
        SmartDashboard.putNumber("anglePIDError", Math.abs(m_pid.getPositionError()));
        SmartDashboard.putNumber("anglePIDOutput", m_activeOutput);
    }

    /**
     * Determines where the angle changer is in its coordinate system by looking at
     * the encoder's output. The output is calculated with
     * {@link AngleConstants}'s {@code convertRotationsToDegrees()}, and rounded to
     * the tenths place.
     *
     * <p>
     * The first time this function is called, the encoder's reported distance is
     * stored as its start position. The position stored there is subtracted from
     * the encoder's distance as an offset on subsequent calls.
     *
     * @return The position of the angle changer as reported by its encoder
     */
    public double getMeasurement() {
        if (startPosition == 0) {
            startPosition = m_encoder.getDistance();
        }
        double currentAngleDegrees = AngleConstants.convertRotationsToDegrees(
                m_encoder.getDistance() - startPosition + AngleConstants.BASE_POS_ROT);

        return 0.1 * Math.round(currentAngleDegrees * 10);
    }

    /**
     * Sets the motor voltage to the result of the PID and FeedForward calculations.
     * This function takes the pidOutput so it can be used as the output function of
     * the {@link PIDCommand} that is this subsystems' default. If enabled is false,
     * then the pid output and the feed forward output are ignored and the motor is
     * told to stop.
     *
     * <p>
     * If the angle changer is at rest, queried using isAtBase, then the feed
     * forward output is not used. This is done so that the angle changer isn't fed
     * voltage by the feed forward while at rest.
     *
     * @param pidOutput The output from the PIDController, passed by the
     *                  {@link PIDCommand}.
     */
    public void useOutput(double pidOutput) {
        double feedOutput = m_feedForward.calculate(Math.toRadians(getMeasurement()), pidOutput);
        double volts = ((m_setpoint == AngleConstants.BASE_POS_DEG && isAtBase()) || !enabled) ? 0 : pidOutput + feedOutput;
        m_sparkMax_right.setVoltage(volts);
        SmartDashboard.putNumber("angleHypotheticalOutput", volts);
        m_sparkMax_right.setVoltage(0);
        m_activeOutput = pidOutput;
    }

    /**
     * Recreates m_pid and m_feedForward based on the PID and SGV constants in
     * {@link AngleConstants}.
     */
    public void updateProportions() {
        double kp = AngleConstants.KP;
        double ki = AngleConstants.KI;
        double kd = AngleConstants.KD;

        double ks = AngleConstants.KS;
        double kg = AngleConstants.KG;
        double kv = AngleConstants.KV;

        m_feedForward = new ArmFeedforward(ks, kg, kv);
        m_pid.setPID(kp, ki, kd);
    }

    /**
     * Resets the {@link PIDController} that this subsystem uses.
     */
    public void pidReset() {
        m_pid.reset();
    }

    /**
     * Sets the setpoint in angles that the angle changer tries to reach. The
     * coordinate system for this is as follows: angle 0 is facing opposite the
     * robot, in the direction that notes are shot, with positive angles causing the
     * angle changer to lower more towards the base.
     *
     * <p>
     * Another way to describe the coordinate system is to give examples: 130 is the
     * angle at which we can shoot into the amp, and 235 is the angle at which we
     * shoot into the speaker, which is the same as the resting position.
     *
     * @param angle The angle that the subsystem will attemt to reach
     */
    public void setSetpoint(double angle) {
        if (angle > AngleConstants.LOWEST_DEG && angle < AngleConstants.HIGHEST_DEG)
            m_setpoint = angle;
    }

    /**
     * Returns the custom angle value stored in {@link SmartDashboard}
     * <p>
     * 
     * @return the angle tied to {@code "testShootAngle"} in SmartDashboard
     */
    public double getSmartDashboardVal() {
        return SmartDashboard.getNumber("testShootAngle", AngleConstants.BASE_POS_DEG);
    }

    /**
     * Sets the right motor's idle mode to coast, releasing the brakes. Also sets
     * the subsystem to be disabled, sending 0 instead of the results of the PID and
     * feed forward to the motors as voltage.
     */
    public void releaseBrakes() {
        m_sparkMax_right.setIdleMode(IdleMode.kCoast);
        enabled = false;
    }

    /**
     * Sets the right motor's idle mode to brake, enabling the brakes. Also sets the
     * subsystem to be enabled, sending the results of the PID and feed forward to
     * the motors as voltage.
     */
    public void engageBrakes() {
        m_sparkMax_right.setIdleMode(IdleMode.kBrake);
        enabled = true;
    }

    /**
     * Returns a {@link PIDCommand} that is intended to be set as the default
     * command of this subsystem. Being a PIDCommand, it never finishes unless
     * interrupted.
     *
     * @param s The {@link AngleSubsystem} that this requires.
     *
     * @return A PIDCommand that runs *this* subsystem, requiring *s*.
     */
    public PIDCommand anglePIDCommand(AngleSubsystem s) {
        return new PIDCommand(m_pid, () -> getMeasurement(), () -> m_setpoint, a -> useOutput(a), s);
    }

    /**
     * Returns whether the absolute position error is less than 5.
     *
     * @return Whether the absolute position error is less than 5
     */
    public boolean isAtSetpoint() {
        return Math.abs(m_pid.getPositionError()) < 5;
    }

    /**
     * Returns whether the absolute difference between the measurement and the start
     * position is less than 2. The start position is defined in
     * {@link AngleConstants} as {@code START_POS_DEG}.
     * 
     * @return Whether the measurment is less than 5 units from the start pos
     */
    public boolean isAtBase() {
        return Math.abs(getMeasurement() - AngleConstants.BASE_POS_DEG) < 2;
    }

    /**
     * Returns whether the absolute difference between the setPos and the
     * measurement is less than 5.
     *
     * @param setPos The setpoint to compare against
     *
     * @return Whether or not the difference between the measurement and the
     *         setpoint is less than 5
     */
    public boolean isAtPosition(double setPos) {
        return Math.abs(getMeasurement() - setPos) < 5;
    }

    /**
     * Returns a {@link Command} which will set the setpoint to the start position
     * upon being scheduled.
     */
    public Command angleToBase() {
        return getSetAngle(() -> AngleConstants.BASE_POS_DEG);
    }

    /**
     * Returns a {@link Command} which will set the setpoint to the highest physical
     * position possible (lowest degrees + 10) upon being scheduled.
     */
    public Command angleToMax() {
        return getSetAngle(() -> AngleConstants.LOWEST_DEG + 10);
    }

    /**
     * Returns a {@link Command} which will set the setpoint to the back stage
     * shooting angle upon being scheduled.
     */
    public Command angleToStageBack() {
        return getSetAngle(() -> AngleConstants.STAGE_BACK_DEG);
    }

    /**
     * Returns a {@link Command} which will set the setpoint to the front stage
     * shooting angle upon being scheduled.
     */
    public Command angleToStageFront() {
        return getSetAngle(() -> AngleConstants.STAGE_FRONT_DEG);
    }

    /**
     * Continuously sets the setpoint to 10 less than what it was when
     * the function was called.
     * 
     * @deprecated
     *             This method has been deprecated due to the trapezoidal motion
     *             profile now utilized
     */
    public Command angleIncrease() {
        return getSetAngle(() -> m_setpoint - 10).repeatedly();
    }

    /**
     * Continuously sets the setpoint to 10 greater than what it was
     * when the function was called.
     * 
     * @deprecated
     *             This method has been deprecated due to the trapezoidal motion
     *             profile now utilized
     */
    public Command angleDecrease() {
        return getSetAngle(() -> m_setpoint + 10).repeatedly();
    }

    /**
     * Returns a Command that sets the angle to the angle specified in
     * {@code "testShootAngle"} in {@link SmartDashboard}.
     */
    public Command getSetSmartDashboard() {
        return getSetAngle(() -> getSmartDashboardVal());
    }

    /**
     * Returns a Command that resets the PID.
     */
    public Command getPIDReset() {
        return Commands.runOnce(() -> pidReset());
    }

    /**
     * Returns a Command that sets the setpoint to the angle provided and finishes
     * when the angle changer is at the setpoint.
     *
     * @param angleSource The angle to set as the setpoint (as a
     *                    {@link DoubleSupplier} in case of non-constant setpoints)
     */
    public Command getSetAngle(DoubleSupplier angleSource) {
        return FactoryCommands.runOnceUntil(() -> {
            setSetpoint(angleSource.getAsDouble());
        }, () -> isAtSetpoint());
    }

    /**
     * This factory command releases the brakes on initialize and then engages the
     * brakes once more when interrupted.
     */
    public Command getReleaseMotors() {
        return startEnd(() -> releaseBrakes(), () -> engageBrakes());
    }
}
