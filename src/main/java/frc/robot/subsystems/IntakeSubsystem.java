package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.IntakeConstants;

/**
 * This class represents an IntakeSubsystem. It utilizes a motor fed with static
 * voltage to move a series of belts. These voltages can be set with
 * {@code pull()}, {@code push()}, and {@code stop()}, each of which having
 * relevant {@link Command} methods.
 * 
 * @author Jacoby (with help for Commands)
 */
public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax_pull = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    /**
     * Creates a new IntakeSubsystem and inverts the motor.
     */
    public IntakeSubsystem() {
        m_sparkMax_pull.setInverted(true);
    }

     /**
     * Sets the motor's voltage to {@link IntakeConstants}'s
     * {@code PULL_VOLTS}.
     */
    public void pull() {
        m_sparkMax_pull.setVoltage(IntakeConstants.PULL_VOLTS);
    }

    /**
     * Sets the motor's voltage to the oppsite of {@link IntakeConstants}'s
     * {@code PUSH_VOLTS}.
     * <p>
     * (The opposite is used here simply for the ease of editing values in the
     * Constants file. It is old code and not worth changing without better reason)
     */
    public void push() {
        m_sparkMax_pull.setVoltage(-IntakeConstants.PUSH_VOLTS);
    }

    /**
     * Sets the motor's voltage to zero.
     */
    public void stop() {
        m_sparkMax_pull.setVoltage(0);
    }

    /**
     * @return a {@link Command} to run the motor in an inward direction until
     *         interrupted,
     *         prompting a stop. This is performed using the {@code pull()} and
     *         {@code stop()} methods of this subsystem.
     */
    public Command getPullStop() {
        return startEnd(() -> pull(), () -> stop());
    }

    /**
     * @return a {@link Command} to run the motor in an outward direction until
     *         interrupted, prompting a stop. This is performed using the
     *         {@code push()} and {@code stop()} methods of this subsystem.
     */
    public Command getPushStop() {
        return runOnce(() -> push());
    }

    /**
     * @return a {@code runOnce()} {@link Command} to stop the motor, using the
     *         inbuilt {@code stop()} method
     */
    public Command getStop() {
        return runOnce(() -> stop());
    }
}
