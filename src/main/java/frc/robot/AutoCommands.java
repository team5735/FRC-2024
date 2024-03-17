package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

public class AutoCommands {
    /**
     * Registers named commands. This makes four commands listed below and registers
     * them using {@link NamedCommands.registerCommands}.
     *
     * <p>
     * <ul>
     * <li>intake.pullCommand() → startIntake
     * <li>intake.stopCommand() → stopIntake
     * <li>Compositions.feedAndShoot(feeder, shooterTop, shooterBottom) →
     * startShooting
     * <li>shooterTop.stopCommand() → stopShooting
     * </ul>
     *
     * @param intake        The intake subsystem. Required by startIntake,
     *                      stopIntake
     * @param feeder        The feeder subsystem. Required by startShooting
     * @param shooterTop    The top shooter subsystem. Required by startShooting,
     *                      stopShooting
     * @param shooterBottom The bottom shooter subsystem. Required by startShooting,
     *                      stopShooting
     */
    static void registerCommands(final IntakeSubsystem intake, final FeederSubsystem feeder,
            final ShooterTopSubsystem shooterTop, final ShooterBottomSubsystem shooterBottom) {
        Map<String, Command> commandsToRegister = new HashMap<>();

        Command startIntake = intake.pullCommand();
        Command stopIntake = intake.stopCommand();
        Command startShooting = Compositions.feedAndShoot(feeder, shooterTop, shooterBottom);
        Command stopShooting = shooterTop.stopCommand();

        commandsToRegister.put("startIntake", startIntake);
        commandsToRegister.put("stopIntake", stopIntake);
        commandsToRegister.put("startShooting", startShooting);
        commandsToRegister.put("stopShooting", stopShooting);

        NamedCommands.registerCommands(commandsToRegister);
    }
}
