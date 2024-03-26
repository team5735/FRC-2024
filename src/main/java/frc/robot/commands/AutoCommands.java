package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Compositions;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

// WAIT!!! BEFORE YOU DO ANYTHING:
// Any command registered in auto for a subsystem that uses a default command should NEVER
// use addRequirements or anything that uses requirements (i.e. instead of subsystem.runOnce()
// you should use Commands.runOnce()) because the default command won't run in auto otherwise!
public class AutoCommands {
    public static void registerCommands(final IntakeSubsystem intake, final FeederSubsystem feeder,
            final ShooterTopSubsystem shooterTop, final ShooterBottomSubsystem shooterBottom) {
        Map<String, Command> commandsToRegister = new HashMap<>();

        commandsToRegister.put("getNote", Compositions.feedNIn(feeder, intake));
        commandsToRegister.put("shooterStart", shooterStart(shooterTop, shooterBottom));

        commandsToRegister.put("stopShooter", stopShooter(shooterTop, shooterBottom));
        commandsToRegister.put("shootNote", new ParallelCommandGroup(feeder.getPullStop(), intake.getPullStop()));
        commandsToRegister.put("waitShootSpinup", spunUpDeadline(shooterTop, shooterBottom));

        NamedCommands.registerCommands(commandsToRegister);
    }

    public static Command getNote(IntakeSubsystem intake, FeederSubsystem feeder) {
        return new SequentialCommandGroup(new ParallelDeadlineGroup(feeder.getPrimeNote(), intake.getPullStop()),
                feeder.getUnprimeNote());
    }

    public static Command shooterStart(ShooterTopSubsystem top, ShooterBottomSubsystem bottom) {
        return new ParallelCommandGroup(
                Commands.runOnce(() -> top.setSetpoint(
                        SmartDashboard.getNumber("shootTopRPM", ShooterConstants.SHOOTER_TOP_DEFAULT_RPM))),
                Commands.runOnce(() -> bottom.setSetpoint(
                        SmartDashboard.getNumber("shootBottomRPM", ShooterConstants.SHOOTER_BOTTOM_DEFAULT_RPM))));
    }

    public static Command stopShooter(ShooterTopSubsystem top, ShooterBottomSubsystem bottom) {
        return new ParallelCommandGroup(Commands.runOnce(() -> top.stop()), Commands.runOnce(() -> bottom.stop()));
    }

    public static Command spunUpDeadline(ShooterTopSubsystem top, ShooterBottomSubsystem bottom) {
        return Commands.waitUntil(() -> top.isSpunUp() && bottom.isSpunUp());
    }
}
