package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FactoryCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FeederConstants;

/**
 * This class represents a FeederSubsystem. It utilizes a motor fed with static
 * voltage to move foam rollers. These voltages can be set with {@code pull()},
 * {@code push()}, and {@code stop()}, each of which having relevant
 * {@link Command} methods.
 * 
 * @author Jacoby (with help for Commands)
 */
public class FeederSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax_pull = new CANSparkMax(Constants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput m_switch = new DigitalInput(Constants.FEEDER_BEAM_PIN);
    private boolean lastBeamBreakStatus = false;

    /**
     * Creates a new FeederSubsystem and inverts the motor.
     */
    public FeederSubsystem() {
        m_sparkMax_pull.setInverted(true);
    }

    /**
     * Sets the motor's voltage to {@link FeederConstants}'s
     * {@code PULL_VOLTS}.
     */
    public void pull() {
        m_sparkMax_pull.setVoltage(FeederConstants.PULL_VOLTS);
    }

    /**
     * Sets the motor's voltage to the oppsite of {@link FeederConstants}'s
     * {@code PUSH_VOLTS}.
     * <p>
     * (The opposite is used here simply for the ease of editing values in the
     * Constants file. It is old code and not worth changing without better reason)
     */
    public void push() {
        m_sparkMax_pull.setVoltage(-FeederConstants.PUSH_VOLTS);
    }

    /**
     * Sets the motor's voltage to zero.
     */
    public void stop() {
        m_sparkMax_pull.setVoltage(0);
    }

    /**
     * Logs values to {@link SmartDashboard}:
     * <p>
     * {@code "feederSwitchStatus"} - The boolean state of the beam break
     * <p>
     * {@code "feederVoltage"} - The bus voltage as read by the motor
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("feederSwitchStatus", getSwitchStatus());
        SmartDashboard.putNumber("feederVoltage", m_sparkMax_pull.getBusVoltage());

        if (lastBeamBreakStatus != getSwitchStatus() && getSwitchStatus() == false) {
            LimelightSubsystem.blinkLedsOnce().schedule();
        }
        lastBeamBreakStatus = getSwitchStatus();
    }

    /**
     * @return the boolean state of the {@link DigitalInput} beam break of this
     *         subsystem
     */
    public boolean getSwitchStatus() {
        return m_switch.get();
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
        return startEnd(() -> push(), () -> stop());
    }

    /**
     * @return a {@link Command} to run the motor in an inward direction until the
     *         {@link DigitalInput} beam break reads false, prompting a stop. This
     *         is performed using the {@code pull()} and {@code stop()} methods of
     *         this subsystem for the motor, and the {@code getSwitchStatus()}
     *         method to read the beam break.
     */
    public Command getPrimeNote() {
        return FactoryCommands.startEndUntil(() -> pull(), () -> stop(), () -> !getSwitchStatus(), this);
    }

    /**
     * @return a {@link Command} to run the motor in an outward direction until the
     *         {@link DigitalInput} beam break reads true, prompting a stop. This
     *         is performed using the {@code push()} and {@code stop()} methods of
     *         this subsystem for the motor, and the {@code getSwitchStatus()}
     *         method to read the beam break.
     */
    public Command getUnprimeNote() {
        return FactoryCommands.startEndUntil(() -> push(), () -> stop(), () -> getSwitchStatus(), this);
    }

    /**
     * @return a {@code runOnce()} {@link Command} to stop the motor, using the
     *         inbuilt {@code stop()} method
     */
    public Command getStop() {
        return runOnce(() -> stop());
    }
}
