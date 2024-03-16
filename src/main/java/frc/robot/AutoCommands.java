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
    static void registerCommands(final IntakeSubsystem intake, final FeederSubsystem feeder,
            final ShooterTopSubsystem shooterTop, final ShooterBottomSubsystem shooterBottom) {
        Map<String, Command> commandsToRegister = new HashMap<>();

        Command startIntake = intake.getPullStop();
        Command stopIntake = intake.getStop();
        Command startShooting = Compositions.feedAndShoot(feeder, shooterTop, shooterBottom);
        Command stopShooting = shooterTop.getStop();

        commandsToRegister.put("startIntake", startIntake);
        commandsToRegister.put("stopIntake", stopIntake);
        commandsToRegister.put("startShooting", startShooting);
        commandsToRegister.put("stopShooting", stopShooting);

        NamedCommands.registerCommands(commandsToRegister);
    }
}
