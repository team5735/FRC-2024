package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.angle.AngleCommandSetAngle;
import frc.robot.commands.feeder.FeederCommandIn;
import frc.robot.commands.feeder.FeederPrimeNote;
import frc.robot.commands.feeder.FeederUnprimeNote;
import frc.robot.commands.intake.IntakeCommandIn;
import frc.robot.commands.shooter.ShooterHoldNStopCommand;
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
    static Command feedAndShoot(FeederSubsystem feeder, ShooterTopSubsystem shooterTop,
            ShooterBottomSubsystem shooterBottom, double topRPM, double bottomRPM) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ShooterSpinUpCommand(shooterTop, shooterBottom, topRPM, bottomRPM),
                        new WaitCommand(0.5)),
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

    public static Command shootNAngleFromFarthest(AngleSubsystem angle, ShooterTopSubsystem top, 
        ShooterBottomSubsystem bottom, FeederSubsystem feeder){
        return new SequentialCommandGroup(
                angle.angleToFarthestSpeaker(),
                feedAndShoot(
                        feeder, top, bottom, ShooterConstants.SHOOTER_TOP_FARTHEST_RPM, 
                        ShooterConstants.SHOOTER_BOTTOM_FARTHEST_RPM
                )
        );
    }

    public static Command feedNIn(FeederSubsystem feeder, IntakeSubsystem intake){
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new FeederPrimeNote(feeder),
                        new IntakeCommandIn(intake)
                ),
                new FeederUnprimeNote(feeder),
                new FeederPrimeNote(feeder)
        );
    }
}
