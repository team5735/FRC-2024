package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.ShooterSpinUpCommand;
import frc.robot.constants.ShooterConstants;
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
     * 
     * <p>
     * This command does not run the intake until 0.5 seconds have passed AND the
     * shooter is at full speed.
     */
    static Command feedAndShootAlsoIntake(FeederSubsystem feeder, IntakeSubsystem intake,
            ShooterTopSubsystem shooterTop,
            ShooterBottomSubsystem shooterBottom, double topRPM, double bottomRPM) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ShooterSpinUpCommand(shooterTop, shooterBottom, topRPM, bottomRPM),
                        new WaitCommand(0.5)),
                new ParallelCommandGroup(
                        feeder.getPullStop(),
                        intake.getPullStop(),
                        shootersHoldNStop(shooterTop, shooterBottom)));
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

    public static Command shootNAngleFromStageBack(AngleSubsystem angle, ShooterTopSubsystem top,
            ShooterBottomSubsystem bottom, FeederSubsystem feeder, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
                angle.angleToStageBack(),
                feedAndShootAlsoIntake(
                        feeder, intake, top, bottom, ShooterConstants.SHOOTER_TOP_STAGE_BACK_RPM,
                        ShooterConstants.SHOOTER_BOTTOM_STAGE_BACK_RPM));
    }

    public static Command shootNAngleFromStageFront(AngleSubsystem angle, ShooterTopSubsystem top,
            ShooterBottomSubsystem bottom, FeederSubsystem feeder, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
                angle.angleToStageFront(),
                feedAndShootAlsoIntake(
                        feeder, intake, top, bottom, ShooterConstants.SHOOTER_TOP_STAGE_FRONT_RPM,
                        ShooterConstants.SHOOTER_BOTTOM_STAGE_FRONT_RPM));
    }

    public static Command feedNIn(FeederSubsystem feeder, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        feeder.getPrimeNote(),
                        intake.getPullStop()),
                feeder.getUnprimeNote(),
                feeder.getPrimeNote());
    }

    /**
     * Returns a command that stops the shooters when interrupted. This does not
     * require either subsystem passed into it.
     *
     * @param shooterTop    The top shooter subsystem, .stop()ed when interruped
     * @param shooterBottom The bottom shooter subsystem, .stop()ed when interruped
     *
     * @return The Command that stops both when interrupted
     */
    public static Command shootersHoldNStop(ShooterTopSubsystem shooterTop, ShooterBottomSubsystem shooterBottom) {
        return Commands.startEnd(() -> {
        }, () -> {
            shooterTop.stop();
            shooterBottom.stop();
        });
    }
}
