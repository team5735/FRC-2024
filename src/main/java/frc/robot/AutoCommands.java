package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.feeder.FeederCommandIn;
import frc.robot.commands.feeder.FeederPrimeNote;
import frc.robot.commands.intake.IntakeCommandIn;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoCommands {
    static void registerCommands(IntakeSubsystem intake, FeederSubsystem feeder) {
        Map<String, Command> commandsToRegister = new HashMap();

        IntakeCommandIn intakeCmd = new IntakeCommandIn(intake);
        FeederPrimeNote feederCmd = new FeederPrimeNote(feeder);
        
        commandsToRegister.put("getNote", new ParallelDeadlineGroup(feederCmd, intakeCmd));

    }
}
