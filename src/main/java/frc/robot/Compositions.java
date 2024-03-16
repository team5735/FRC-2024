package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.feeder.FeederCommandIn;
import frc.robot.commands.shooter.ShooterHoldNStopCommand;
import frc.robot.commands.shooter.ShooterSpinUpCommand;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

/**
 * A collection of composition commands which don't have a clear subsystem they
 * belong to
 */
public class Compositions {
    /*
     * Creates and returns a new SequentialCommandGroup that first spins up the
     * shooter, that is to say it gets the shooter to full speed, and then has a
     * ParallelCommandGroup that feeds the NOTE in, and simultaneously keeps the
     * shooter at full speed.
     */
    static Command feedAndShoot(FeederSubsystem feeder, ShooterTopSubsystem shooterTop,
            ShooterBottomSubsystem shooterBottom) {
        return new SequentialCommandGroup(
                new ShooterSpinUpCommand(shooterTop, shooterBottom),
                new ParallelCommandGroup(
                        new FeederCommandIn(feeder),
                        new ShooterHoldNStopCommand(shooterTop, shooterBottom)));
    }

    public static Command angleUpdateWithIntake(Command angleSetCommand, AngleSubsystem angler,
            IntakeSubsystem intake) {
        return (angler.isAtBase())
                ? new ParallelCommandGroup(
                        angleSetCommand,
                        new ParallelDeadlineGroup(
                                new WaitCommand(2),
                                intake.getPullStop()))
                : angleSetCommand;
    }
}
