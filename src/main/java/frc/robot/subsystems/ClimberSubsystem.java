package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.util.NTDoubleSection;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_sparkMax;
    private final RelativeEncoder m_encoder;

    NTDoubleSection m_doubles;

    public ClimberSubsystem(String name, int motorID) {
        m_sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
        m_encoder = m_sparkMax.getEncoder();
        m_doubles = new NTDoubleSection(name, "output", "position");
    }

    private void up() {
        m_sparkMax.setVoltage(-ClimberConstants.CLIMBER_UP_VOLTS);
    }

    private void down() {
        m_sparkMax.setVoltage(ClimberConstants.CLIMBER_DOWN_VOLTS);
    }

    private void stop() {
        m_sparkMax.setVoltage(0);
    }

    @Override
    public void periodic() {
        m_doubles.set("output", m_sparkMax.getOutputCurrent());
        m_doubles.set("position", m_encoder.getPosition());
    }

    public Command getUpStop() {
        return startEnd(() -> up(), () -> stop());
    }

    public Command getDownStop() {
        return startEnd(() -> down(), () -> stop());
    }

    public Command getStop() {
        return runOnce(() -> stop());
    }
}
