// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.angle.AngleCommandPIDReset;
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
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    private final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();
    private final DrivetrainSubsystem m_drivetrain = TunerConstants.DriveTrain;

    // Programming war crime :3
    private static boolean m_isFieldCentric = true;
    public static Supplier<Boolean> m_getFieldCentric = () -> m_isFieldCentric;
    // private final Telemetry m_telemetry = new Telemetry(.1);

    private double m_turboMultiplier = 10;
    private double m_normalMultiplier = 2;
    private double m_slowMultiplier = 1;

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        // m_drivetrain.registerTelemetry(m_telemetry::telemeterize);
        // Configure the trigger bindings
        configureBindings();
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
        // Schedule `exampleMethodCommand` when the Xbox controller's B button
        // is pressed, cancelling on release.

        m_drivingController.b().whileTrue(new BrakeCommand(m_drivetrain));
        // m_drivingController.b().whileTrue(m_drivetrain.applyRequest(() ->
        // m_pointWheelsAt
        // .withModuleDirection(new Rotation2d(m_drivingController.getLeftY(),
        // m_drivingController.getLeftX()))));
        m_drivingController.x()
                .whileTrue(new LimelightAimCommandV2(m_limelightSubsystem, m_drivetrain, m_angleSubsystem));
        m_drivingController.y().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

        m_drivingController.leftBumper()
                .whileTrue(new ParallelCommandGroup(new IntakeCommandOut(m_intakeSubsystem),
                        new FeederCommandOut(m_feederSubsystem)));
        m_drivingController.rightBumper()
                .whileTrue(new ParallelCommandGroup(new IntakeCommandIn(m_intakeSubsystem),
                        new FeederCommandIn(m_feederSubsystem)));

        m_drivingController.povUp().onTrue(m_candleSubsystem.colorReady());
        m_drivingController.povUpRight().onTrue(m_candleSubsystem.colorAuto());
        m_drivingController.povRight().onTrue(m_candleSubsystem.colorAiming());
        m_drivingController.povDownRight().onTrue(m_candleSubsystem.colorAimed());
        m_drivingController.povDown().onTrue(m_candleSubsystem.colorShooting());
        m_drivingController.povDownLeft().onTrue(m_candleSubsystem.colorIntakeRunning());

        m_drivingController.povLeft().whileTrue(m_drivetrain.nyoom());

        m_drivetrain.setDefaultCommand(new DriveCommand(m_drivetrain, () -> deadband(m_drivingController.getLeftX()),
                () -> m_drivingController.povUp().getAsBoolean() ? -1.0 : deadband(m_drivingController.getLeftY()),
                () -> {
                    return deadband(
                            m_drivingController.getLeftTriggerAxis() - m_drivingController.getRightTriggerAxis());
                }, () -> {
                    return m_drivingController.leftStick().getAsBoolean() ? m_slowMultiplier
                            : (m_drivingController.a().getAsBoolean() ? m_turboMultiplier : m_normalMultiplier);
                }));

        // some lines were not copied from the drivetrain

        m_subsystemController.a().whileTrue(new ShooterCommand(m_shooterSubsystem));
        // m_subsystemController.b().whileTrue(new IntakeCommandIn(m_intakeSubsystem));
        m_subsystemController.b().whileTrue(new AngleCommandReleaseMotors(m_angleSubsystem));
        // m_subsystemController.x().onTrue(new FeederPrimeNote(m_feederSubsystem));
        m_subsystemController.y().whileTrue(new FeederCommandIn(m_feederSubsystem));
        m_subsystemController.y().onTrue(new AngleCommandSetAngle(m_angleSubsystem));

        m_subsystemController.start().onTrue(new AngleCommandPIDReset(m_angleSubsystem));

        m_subsystemController.leftBumper().whileTrue(new ClimberCommandLeftUp(m_climberSubsystem));
        m_subsystemController.rightBumper().whileTrue(new ClimberCommandRightUp(m_climberSubsystem));
        m_subsystemController.rightTrigger(0.1).whileTrue(new ClimberCommandRightDown(m_climberSubsystem));
        m_subsystemController.leftTrigger(0.1).whileTrue(new ClimberCommandLeftDown(m_climberSubsystem));
    }

    // activates the useOutput() methods of PID-implemented subsystems 
    // (Please let Jacoby know if you have a better way of doing this)
    public void useSubsystemOutputs(){
        m_angleSubsystem.useOutput();
        m_shooterSubsystem.useOutput();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.auto(m_limelightSubsystem, m_drivetrain, m_candleSubsystem);
    }
}
