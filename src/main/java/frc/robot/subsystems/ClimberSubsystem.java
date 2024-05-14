package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.util.NTDoubleSection;

/**
 * This class represents a ClimberSubsystem. It utilizes the in-built break mode
 * of a motor to hold positions and static voltages to move up or down. These
 * voltages can be set with {@code up()}, {@code down()}, and {@code stop()},
 * each of which having relevant {@link Command} methods.
 * 
 * @author Jacoby (with additional help on Commands and the
 *         {@link NetworkTable})
 */
public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax;
    private final RelativeEncoder m_encoder;

    NTDoubleSection m_doubles;

    /**
     * Creates a new ClimberSubsystem and logs the motor's encoder
     * 
     * @param name    the name of the {@link NTDoubleSection} to be created
     * @param motorID the ID of the {@link CANSparkMax} motor controller to be tied
     *                to this subsystem
     */
    public ClimberSubsystem(String name, int motorID) {
        m_sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
        m_encoder = m_sparkMax.getEncoder();
        m_doubles = new NTDoubleSection(name, "output", "position");
    }

    /**
     * Sets the motor's voltage to the opposite of {@link ClimberConstants}'s
     * {@code UP_VOLTS}.
     * <p>
     * (The opposite is used here simply for the ease of editing values in the
     * Constants file. It is old code and not worth changing without better reason)
     */
    private void up() {
        m_sparkMax.setVoltage(-ClimberConstants.UP_VOLTS);
    }

    /**
     * Sets the motor's voltage to {@link ClimberConstants}'s
     * {@code DOWN_VOLTS}.
     */
    private void down() {
        m_sparkMax.setVoltage(ClimberConstants.DOWN_VOLTS);
    }

    /**
     * Sets the motor's voltage to zero, stopping the Climber, provided the motor's
     * breaks function.
     */
    private void stop() {
        m_sparkMax.setVoltage(0);
    }

    /**
     * Logs values to the {@link NTDoubleSection}:
     * <p>
     * {@code "output"} - The output current to the motor
     * <p>
     * {@code "position"} - The position read by the motor's encoder
     */
    @Override
    public void periodic() {
        m_doubles.set("output", m_sparkMax.getOutputCurrent());
        m_doubles.set("position", m_encoder.getPosition());
    }

    /**
     * @return a {@link Command} to run the motor in an upward direction until
     *         interrupted,
     *         prompting a stop. This is performed using the {@code up()} and
     *         {@code stop()} methods of this subsystem.
     */
    public Command getUpStop() {
        return startEnd(() -> up(), () -> stop());
    }

    /**
     * @return a {@link Command} to run the motor in a downward direction until
     *         interrupted, prompting a stop. This is performed using the
     *         {@code down()} and {@code stop()} methods of this subsystem.
     */
    public Command getDownStop() {
        return startEnd(() -> down(), () -> stop());
    }

    /**
     * @return a {@code runOnce()} {@link Command} to stop the motor, using the
     *         inbuilt {@code stop()} method
     */
    public Command getStop() {
        return runOnce(() -> stop());
    }
}
