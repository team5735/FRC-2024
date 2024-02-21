package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    private final Supplier<Double> m_stickX;
    private final Supplier<Double> m_stickY;
    private final Supplier<Double> m_rotate;
    private final Supplier<Double> m_multiplier;
    private Watchdog m_watchdog = new Watchdog(0.02, () -> {});
    public DriveCommand(DrivetrainSubsystem drivetrain, Supplier<Double> stickX, Supplier<Double> stickY, Supplier<Double> rotate, Supplier<Double> multiplier) {
        m_drivetrain = drivetrain;

        m_stickX = stickX;
        m_stickY = stickY;
        m_rotate = rotate;
        m_multiplier = multiplier;

        addRequirements(m_drivetrain);
    }

    public void execute() {
        m_watchdog.reset();
        m_drivetrain.drive(-m_stickY.get(), -m_stickX.get(), m_rotate.get(), m_multiplier.get());
        m_watchdog.disable();
        if (m_watchdog.isExpired()) {
            System.out.println("watchdog expired :( " + m_watchdog.getTimeout() + ", " + m_watchdog.getTime());
        }
    }
}
