package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.feeder.FeederCommandIn;
import frc.robot.commands.feeder.FeederCommandOut;
import frc.robot.commands.feeder.FeederPrimeNote;
import frc.robot.commands.intake.IntakeCommandIn;
import frc.robot.commands.shooter.ShooterSpinUpCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

public class AutoCommands {
    static void registerCommands(IntakeSubsystem intake, FeederSubsystem feeder, ShooterTopSubsystem top, ShooterBottomSubsystem bot) {
        Map<String, Command> commandsToRegister = new HashMap<>();

        IntakeCommandIn intakeCmd = new IntakeCommandIn(intake);
        FeederPrimeNote feederInCmd = new FeederPrimeNote(feeder);
        FeederCommandOut feederOutCmd = new FeederCommandOut(feeder);
        ShooterSpinUpCommand shootCmd = new ShooterSpinUpCommand(top, bot);
                
        commandsToRegister.put("getNote", new ParallelDeadlineGroup(feederInCmd, intakeCmd));
        commandsToRegister.put("primeIt", new ParallelCommandGroup(feederInCmd, intakeCmd, shootCmd));
        commandsToRegister.put("shoot", new ParallelCommandGroup(feederOutCmd, shootCmd));

        NamedCommands.registerCommands(commandsToRegister);
    }
}
