// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.drivetrain.BrakeCommand;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.limelight.LimelightAimCommand;
import frc.robot.commands.shooter.ShooterSpinUpCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
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
    private final ClimberSubsystem m_climberLeftSubsystem = new ClimberSubsystem("left climber",
            Constants.CLIMBER_MOTOR_LEFT_ID);
    private final ClimberSubsystem m_climberRightSubsystem = new ClimberSubsystem("right climber",
            Constants.CLIMBER_MOTOR_RIGHT_ID);
    private final DrivetrainSubsystem m_drivetrain = TunerConstants.DriveTrain;

    // Programming war crime :3
    private static boolean m_isFieldCentric = true;
    public static Supplier<Boolean> m_getFieldCentric = () -> m_isFieldCentric;
    // private final Telemetry m_telemetry = new Telemetry(.1);

    private double m_slowMultiplier = DrivetrainConstants.SLOW_SPEED;
    private double m_normalMultiplier = DrivetrainConstants.NORMAL_SPEED;
    private double m_turboMultiplier = DrivetrainConstants.TURBO_SPEED;

    private final SendableChooser<Command> m_autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        // m_drivetrain.registerTelemetry(m_telemetry::telemeterize);
        // Configure the trigger bindings
        AutoCommands.registerCommands(m_intakeSubsystem, m_feederSubsystem, m_shooterTopSubsystem,
                m_shooterBottomSubsystem);
        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("pick an auto", m_autoChooser);

        configureBindings();
    }

    private static double deadband(double input) {
        if (Math.abs(input) <= DrivetrainConstants.DEADBAND) {
            return 0;
        }
        return input;
    }

    /**
     * Use this method to define your trigger → command mappings. Triggers can be
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
        m_drivingController.leftBumper().whileTrue(new ParallelCommandGroup(
                m_intakeSubsystem.getPushStop(),
                m_feederSubsystem.getPushStop()));
        m_drivingController.rightBumper()
                .whileTrue(Compositions.feedNIn(m_feederSubsystem, m_intakeSubsystem));

        m_drivingController.start().onTrue(Commands.runOnce(() -> updateMultipliers()));

        m_drivetrain.setDefaultCommand(
                new DriveCommand(m_drivetrain,
                        () -> -deadband(m_drivingController.getLeftX()),
                        () -> -deadband(m_drivingController.getLeftY()),
                        () -> {
                            return deadband(
                                    m_drivingController.getLeftTriggerAxis()
                                            - m_drivingController
                                                    .getRightTriggerAxis());
                        },
                        () -> {
                            return m_drivingController.getHID().getRightStickButton()
                                    ? m_turboMultiplier
                                    : (m_drivingController.getHID()
                                            .getLeftStickButton()
                                                    ? m_slowMultiplier
                                                    : m_normalMultiplier);
                        }));

        m_drivingController.a().whileTrue(
                Compositions.feedAndShootAlsoIntake(
                        m_feederSubsystem, m_intakeSubsystem, m_shooterTopSubsystem,
                        m_shooterBottomSubsystem,
                        SmartDashboard.getNumber("shootTopRPM",
                                ShooterConstants.SHOOTER_TOP_DEFAULT_RPM),
                        SmartDashboard.getNumber("shootBottomRPM",
                                ShooterConstants.SHOOTER_BOTTOM_DEFAULT_RPM)));

        m_drivingController.x().whileTrue(
                new LimelightAimCommand(m_limelightSubsystem, m_drivetrain, m_angleSubsystem));
        m_drivingController.y().onTrue(Commands.runOnce(() -> {
            m_drivetrain.seedFieldRelative();
            m_drivetrain.getPigeon2().setYaw(0);
        }, m_drivetrain));

        m_drivingController.povUp().onTrue(
                Compositions.angleUpdateWithIntake(m_angleSubsystem.angleToMax(), m_angleSubsystem,
                        m_intakeSubsystem));
        m_drivingController.povDown().onTrue(
                m_angleSubsystem.angleToBase());

        m_drivingController.povLeft().onTrue(m_angleSubsystem.getSetAngle(180));

        // some lines were not copied from the drivetrain

        m_subsystemController.a().whileTrue(
                Compositions.feedAndShootAlsoIntake(
                        m_feederSubsystem, m_intakeSubsystem, m_shooterTopSubsystem,
                        m_shooterBottomSubsystem,
                        SmartDashboard.getNumber("shootTopRPM",
                                ShooterConstants.SHOOTER_TOP_DEFAULT_RPM),
                        SmartDashboard.getNumber("shootBottomRPM",
                                ShooterConstants.SHOOTER_BOTTOM_DEFAULT_RPM)));

        m_drivingController.povRight()
                .whileTrue(new ParallelCommandGroup(
                        m_angleSubsystem.getSetSmartDashboard(),
                        new SequentialCommandGroup(
                                new ShooterSpinUpCommand(
                                        m_shooterTopSubsystem, m_shooterBottomSubsystem,
                                        ShooterConstants.SHOOTER_TOP_DEFAULT_RPM,
                                        ShooterConstants.SHOOTER_BOTTOM_DEFAULT_RPM),
                                new ParallelDeadlineGroup(
                                        m_feederSubsystem.getPullStop(),
                                        Compositions.shootersHoldNStop(m_shooterTopSubsystem,
                                                m_shooterBottomSubsystem)))));

        m_subsystemController.y().whileTrue(Compositions.shootNAngleFromStageBack(
                m_angleSubsystem, m_shooterTopSubsystem, m_shooterBottomSubsystem, m_feederSubsystem,
                m_intakeSubsystem));
        m_subsystemController.x().whileTrue(new ParallelCommandGroup(
                m_intakeSubsystem.getPushStop(),
                m_feederSubsystem.getPushStop()));
        m_subsystemController.a().whileTrue(m_limelightSubsystem.blinkLeds());

        m_subsystemController.leftBumper().whileTrue(m_climberLeftSubsystem.getUpStop());
        m_subsystemController.rightBumper().whileTrue(m_climberRightSubsystem.getUpStop());
        m_subsystemController.leftTrigger(0.1).whileTrue(m_climberLeftSubsystem.getDownStop());
        m_subsystemController.rightTrigger(0.1).whileTrue(m_climberRightSubsystem.getDownStop());

        m_angleSubsystem.setDefaultCommand(m_angleSubsystem.anglePIDCommand(m_angleSubsystem));
        m_shooterTopSubsystem.setDefaultCommand(m_shooterTopSubsystem.shootPIDCommand());
        m_shooterBottomSubsystem
                .setDefaultCommand(m_shooterBottomSubsystem.shootPIDCommand());
    }

    private void updateMultipliers() {
        m_slowMultiplier = SmartDashboard.getNumber("drivetrain_slowSpeed", m_slowMultiplier);
        m_normalMultiplier = SmartDashboard.getNumber("drivetrain_normalSpeed", m_normalMultiplier);
        m_turboMultiplier = SmartDashboard.getNumber("drivetrain_turboSpeed", m_turboMultiplier);
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
        return auto;
    }

    public void resetShenanigans() {
        m_shooterTopSubsystem.setSetpoint(0);
        m_shooterBottomSubsystem.setSetpoint(0);
        m_angleSubsystem.setSetpoint(235);
    }
}
