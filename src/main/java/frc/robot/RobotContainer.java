// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrain.BrakeCommand;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.TunableNumber;

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

    private final DrivetrainSubsystem m_drivetrain = TunerConstants.DriveTrain;
    private final VisionSubsystem vision = new VisionSubsystem(m_drivetrain);

    // Programming war crime :3
    private static boolean m_isFieldCentric = true;
    public static Supplier<Boolean> m_getFieldCentric = () -> m_isFieldCentric;
    private final Telemetry m_telemetry = new Telemetry(.1);

    private double m_slowMultiplier = DrivetrainConstants.SLOW_SPEED;
    private double m_normalMultiplier = DrivetrainConstants.NORMAL_SPEED;
    private double m_turboMultiplier = DrivetrainConstants.TURBO_SPEED;

    private final SendableChooser<Command> m_autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        m_drivetrain.registerTelemetry(m_telemetry::telemeterize);
        // Configure the trigger bindings
        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("pick an auto", m_autoChooser);

        configureDriverBindings();

        m_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
    }

    private static double deadband(double input) {
        if (Math.abs(input) <= DrivetrainConstants.DEADBAND) {
            return 0;
        }
        return input;
    }

    TunableNumber turningTarget = new TunableNumber("turn target");
    TunableNumber transTargetX = new TunableNumber("trans target x");
    TunableNumber transTargetY = new TunableNumber("trans target y");

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
    private void configureDriverBindings() {
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

        this.vision.setDefaultCommand(Commands.idle(this.vision));

        m_drivingController.a()
                .onTrue(Compositions.visionTransRot(m_drivetrain, () -> turningTarget.get(),
                        () -> this.m_drivetrain.getEstimatedPosition().getTranslation()));
        m_drivingController.b().onTrue(this.vision.getSeedPigeon());

        m_drivingController.y().onTrue(m_drivetrain.runOnce(() -> {
            m_drivetrain.seedFieldRelative();
            m_drivetrain.getPigeon2().setYaw(0);
            m_drivetrain.getPigeon2().reset();
        }));
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
}
