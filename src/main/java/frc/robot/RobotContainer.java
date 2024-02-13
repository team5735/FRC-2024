// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.climber.ClimberCommandLeftDown;
import frc.robot.commands.climber.ClimberCommandLeftStop;
import frc.robot.commands.climber.ClimberCommandLeftUp;
import frc.robot.commands.climber.ClimberCommandRightDown;
import frc.robot.commands.climber.ClimberCommandRightStop;
import frc.robot.commands.climber.ClimberCommandRightUp;
import frc.robot.commands.feeder.FeederCommandIn;
import frc.robot.commands.feeder.FeederCommandStop;
import frc.robot.commands.intake.IntakeCommandIn;
import frc.robot.commands.intake.IntakeCommandStop;
import frc.robot.commands.limelight.LimelightAimCommandOld;
import frc.robot.commands.limelight.LimelightAimCommandNew;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.ShooterCommandStop;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
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
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    private final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
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

        m_driverController.a().whileTrue(new ShooterCommand(m_shooterSubsystem))
                .whileFalse(new ShooterCommandStop(m_shooterSubsystem));

        m_driverController.b().whileTrue(new IntakeCommandIn(m_intakeSubsystem))
                .whileFalse(new IntakeCommandStop(m_intakeSubsystem));

        m_driverController.x().whileTrue(new LimelightAimCommandNew(m_limelightSubsystem));

        m_driverController.y().whileTrue(new FeederCommandIn(m_feederSubsystem))
                .whileFalse(new FeederCommandStop(m_feederSubsystem));

        // climbing :3
        m_driverController.rightBumper().whileTrue(new ClimberCommandRightUp(m_climberSubsystem))
                .whileFalse(new ClimberCommandRightStop(m_climberSubsystem));
        m_driverController.leftBumper().whileTrue(new ClimberCommandLeftUp(m_climberSubsystem))
                .whileFalse(new ClimberCommandLeftStop(m_climberSubsystem));

        m_driverController.rightTrigger(0.5).whileTrue(new ClimberCommandRightDown(m_climberSubsystem))
                .whileFalse(new ClimberCommandRightStop(m_climberSubsystem));
        m_driverController.leftTrigger(0.5).whileTrue(new ClimberCommandLeftDown(m_climberSubsystem))
                .whileFalse(new ClimberCommandLeftStop(m_climberSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.auto(m_limelightSubsystem, m_candleSubsystem);
    }
}
