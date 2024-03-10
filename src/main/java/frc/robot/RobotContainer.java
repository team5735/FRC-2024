// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.angle.AngleCommandReleaseMotors;
import frc.robot.commands.angle.AngleCommandSetAngle;
import frc.robot.commands.climber.ClimberCommandLeftDown;
import frc.robot.commands.climber.ClimberCommandLeftUp;
import frc.robot.commands.climber.ClimberCommandRightDown;
import frc.robot.commands.climber.ClimberCommandRightUp;
import frc.robot.commands.drivetrain.BrakeCommand;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.feeder.FeederCommandIn;
import frc.robot.commands.feeder.FeederCommandOut;
import frc.robot.commands.feeder.FeederPrimeNote;
import frc.robot.commands.intake.IntakeCommandIn;
import frc.robot.commands.intake.IntakeCommandOut;
import frc.robot.commands.limelight.LimelightAimCommandV2;
import frc.robot.commands.limelight.SetStartingPoseCommand;
import frc.robot.commands.shooter.ShooterHoldNStopCommand;
import frc.robot.commands.shooter.ShooterSpinUpCommand;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.climber.ClimberLeftSubsystem;
import frc.robot.subsystems.climber.ClimberRightSubsystem;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_drivingController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController m_subsystemController = new CommandXboxController(
            OperatorConstants.SUBSYSTEM_CONTROLLER_PORT);

    private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final AngleSubsystem m_angleSubsystem = new AngleSubsystem();
    private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();
    private final ShooterTopSubsystem m_shooterTopSubsystem = new ShooterTopSubsystem();
    private final ShooterBottomSubsystem m_shooterBottomSubsystem = new ShooterBottomSubsystem();
    private final ClimberLeftSubsystem m_climberLeftSubsystem = new ClimberLeftSubsystem();
    private final ClimberRightSubsystem m_climberRightSubsystem = new ClimberRightSubsystem();
    private final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();
    private final DrivetrainSubsystem m_drivetrain = TunerConstants.DriveTrain;

    // Programming war crime :3
    private static boolean m_isFieldCentric = false;
    public static Supplier<Boolean> m_getFieldCentric = () -> m_isFieldCentric;
    // private final Telemetry m_telemetry = new Telemetry(.1);

    private double m_turboMultiplier = 10;
    private double m_normalMultiplier = 2;
    private double m_slowMultiplier = 1;

    private final SendableChooser<Command> m_autoChooser = AutoBuilder.buildAutoChooser();

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        // m_drivetrain.registerTelemetry(m_telemetry::telemeterize);
        // Configure the trigger bindings
        configureBindings();

        SmartDashboard.putData("pick an auto", m_autoChooser);
    }

    private static double deadband(double input) {
        if (Math.abs(input) <= DrivetrainConstants.DEADBAND) {
            return 0;
        }
        return input;
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // A is bound to acceleration
        m_drivingController.b().whileTrue(new BrakeCommand(m_drivetrain));
        m_drivingController.x()
                .whileTrue(new LimelightAimCommandV2(m_limelightSubsystem, m_drivetrain, m_angleSubsystem));
        m_drivingController.y().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

        m_drivingController.leftBumper()
                .whileTrue(new ParallelCommandGroup(new IntakeCommandOut(m_intakeSubsystem),
                        new FeederCommandOut(m_feederSubsystem)));
        m_drivingController.rightBumper()
                .whileTrue(new ParallelDeadlineGroup(new FeederPrimeNote(m_feederSubsystem), new IntakeCommandIn(m_intakeSubsystem)));

        m_drivingController.povUp().onTrue(m_candleSubsystem.colorReady());
        m_drivingController.povUpRight().onTrue(m_candleSubsystem.colorAuto());
        m_drivingController.povRight().onTrue(m_candleSubsystem.colorAiming());
        m_drivingController.povDownRight().onTrue(m_candleSubsystem.colorAimed());
        m_drivingController.povDown().onTrue(m_candleSubsystem.colorShooting());
        m_drivingController.povDownLeft().onTrue(m_candleSubsystem.colorIntakeRunning());

        m_drivingController.povLeft().whileTrue(m_drivetrain.nyoom());

        m_drivetrain.setDefaultCommand(new DriveCommand(m_drivetrain, () -> -deadband(m_drivingController.getLeftX()),
                () -> -deadband(m_drivingController.getLeftY()),
                () -> {
                    return deadband(
                            m_drivingController.getLeftTriggerAxis() - m_drivingController.getRightTriggerAxis());
                }, () -> {
                    return m_drivingController.getHID().getLeftStickButton() ? m_slowMultiplier
                            : (m_drivingController.getHID().getAButton() ? m_turboMultiplier : m_normalMultiplier);
                }));

        // some lines were not copied from the drivetrain

        m_subsystemController.a()
                .whileTrue(feedNShoot(m_feederSubsystem, m_shooterTopSubsystem, m_shooterBottomSubsystem));
        m_subsystemController.b().whileTrue(new AngleCommandReleaseMotors(m_angleSubsystem));
        m_subsystemController.x().onTrue(new FeederPrimeNote(m_feederSubsystem));
        m_subsystemController.y().onTrue(new AngleCommandSetAngle(m_angleSubsystem));

        m_subsystemController.leftBumper().whileTrue(new ClimberCommandLeftUp(m_climberLeftSubsystem));
        m_subsystemController.rightBumper().whileTrue(new ClimberCommandRightUp(m_climberRightSubsystem));
        m_subsystemController.rightTrigger(0.1).whileTrue(new ClimberCommandRightDown(m_climberRightSubsystem));
        m_subsystemController.leftTrigger(0.1).whileTrue(new ClimberCommandLeftDown(m_climberLeftSubsystem));

        m_angleSubsystem.setDefaultCommand(m_angleSubsystem.anglePidCommand(m_angleSubsystem));
        m_shooterTopSubsystem.setDefaultCommand(m_shooterTopSubsystem.shootPidCommand(m_shooterTopSubsystem));
        m_shooterBottomSubsystem.setDefaultCommand(m_shooterBottomSubsystem.shootPidCommand(m_shooterBottomSubsystem));
    }

    private Command feedNShoot(FeederSubsystem feeder, ShooterTopSubsystem shootTop,
            ShooterBottomSubsystem shootBottom) {
        return new SequentialCommandGroup(
                new ShooterSpinUpCommand(shootTop, shootBottom),
                new ParallelCommandGroup(new FeederCommandIn(feeder),
                        new ShooterHoldNStopCommand(shootTop, shootBottom)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        Command auto = m_autoChooser.getSelected();
        if (auto == null) {
            System.out.println("auto is null");
            return new BrakeCommand(m_drivetrain);
        }

        // we need to get the starting pose from the Limelight
        SequentialCommandGroup group = new SequentialCommandGroup(
                new SetStartingPoseCommand(m_drivetrain, m_limelightSubsystem),
                auto);
        return group;
    }
}
