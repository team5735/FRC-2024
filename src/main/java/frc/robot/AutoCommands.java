package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.feeder.FeederPrimeNote;
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

        commandsToRegister.put("getNote", getNote(intake, feeder));
        commandsToRegister.put("spinUpShooter", spinUpShooter(shooterTop, shooterBottom));
        commandsToRegister.put("stopShooter", stopShooter(shooterTop, shooterBottom));
        commandsToRegister.put("shootNote", feeder.getPull());

        NamedCommands.registerCommands(commandsToRegister);
    }

    public static Command getNote(IntakeSubsystem intake, FeederSubsystem feeder) {
        return new ParallelDeadlineGroup(new FeederPrimeNote(feeder), intake.getPullStop());
    }

    public static Command spinUpShooter(ShooterTopSubsystem top, ShooterBottomSubsystem bottom) {
        return new ParallelCommandGroup(top.runOnce(() -> top.start()), bottom.runOnce(() -> bottom.start()));
    }

    public static Command stopShooter(ShooterTopSubsystem top, ShooterBottomSubsystem bottom) {
        return new ParallelCommandGroup(top.runOnce(() -> top.stop()), bottom.runOnce(() -> bottom.stop()));
    }
}
