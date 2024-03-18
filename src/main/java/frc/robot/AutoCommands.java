package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.feeder.FeederPrimeNote;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

// WAIT!!! BEFORE YOU DO ANYTHING:
// Any command registered in auto for a subsystem that uses a default command should NEVER
// use addRequirements or anything that uses requirements (i.e. instead of subsystem.runOnce()
// you should use Commands.runOnce()) because the default command won't run in auto otherwise!
public class AutoCommands {
    static void registerCommands(final IntakeSubsystem intake, final FeederSubsystem feeder,
            final ShooterTopSubsystem shooterTop, final ShooterBottomSubsystem shooterBottom) {
        Map<String, Command> commandsToRegister = new HashMap<>();

        // Command startIntake = intake.getPullStop();
        // Command stopIntake = intake.getStop();
        // Command startShooting = Compositions.feedAndShoot(feeder, shooterTop,
        // shooterBottom);
        // Command stopShooting = shooterTop.getStop();

        // commandsToRegister.put("startIntake", startIntake);
        // commandsToRegister.put("stopIntake", stopIntake);
        // commandsToRegister.put("startShooting", startShooting);
        // commandsToRegister.put("stopShooting", stopShooting);

        commandsToRegister.put("getNote", getNote(intake, feeder));
        commandsToRegister.put("shooterStart", shooterStart(shooterTop, shooterBottom));

        commandsToRegister.put("stopShooter", stopShooter(shooterTop, shooterBottom));
        commandsToRegister.put("shootNote", feeder.getPull());

        NamedCommands.registerCommands(commandsToRegister);
    }

    public static Command getNote(IntakeSubsystem intake, FeederSubsystem feeder) {
        return new ParallelDeadlineGroup(new FeederPrimeNote(feeder), intake.getPullStop());
    }

    public static Command shooterStart(ShooterTopSubsystem top, ShooterBottomSubsystem bottom) {
        return new ParallelCommandGroup(Commands.runOnce(() -> top.start()), Commands.runOnce(() -> bottom.start()));
    }

    public static Command stopShooter(ShooterTopSubsystem top, ShooterBottomSubsystem bottom) {
        return new ParallelCommandGroup(Commands.runOnce(() -> top.stop()), Commands.runOnce(() -> bottom.stop()));
    }

    public static Command spunUpDeadline(ShooterTopSubsystem top, ShooterBottomSubsystem bottom){
        return Commands.waitUntil(() -> top.isSpunUp() && bottom.isSpunUp());
    }
}
