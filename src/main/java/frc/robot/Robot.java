// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AngleConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button
        // bindings, and put our autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        SmartDashboard.putNumber("angleKP", AngleConstants.ANGLE_KP);
        SmartDashboard.putNumber("angleKI", AngleConstants.ANGLE_KI);
        SmartDashboard.putNumber("angleKD", AngleConstants.ANGLE_KD);

        SmartDashboard.putNumber("angleKS", AngleConstants.ANGLE_KS);
        SmartDashboard.putNumber("angleKG", AngleConstants.ANGLE_KG);
        SmartDashboard.putNumber("angleKV", AngleConstants.ANGLE_KV);

        SmartDashboard.putNumber("anglePos", 0);


        SmartDashboard.putNumber("climbRightUpVoltage", ClimberConstants.CLIMBER_RIGHT_UP_VOLTS);
        SmartDashboard.putNumber("climbLeftUpVoltage", ClimberConstants.CLIMBER_LEFT_UP_VOLTS);
        SmartDashboard.putNumber("climbRightDownVoltage", ClimberConstants.CLIMBER_RIGHT_DOWN_VOLTS);
        SmartDashboard.putNumber("climbLeftDownVoltage", ClimberConstants.CLIMBER_LEFT_DOWN_VOLTS);

        
        SmartDashboard.putNumber("feederPullVoltage", FeederConstants.FEEDER_PULL_VOLTS);
        SmartDashboard.putBoolean("feederSwitchStatus", false);


        SmartDashboard.putNumber("intakeKP", IntakeConstants.INTAKE_KP);
        SmartDashboard.putNumber("intakeKI", IntakeConstants.INTAKE_KI);
        SmartDashboard.putNumber("intakeKD", IntakeConstants.INTAKE_KD);

        SmartDashboard.putNumber("intakeKS", IntakeConstants.INTAKE_KS);
        SmartDashboard.putNumber("intakeKV", IntakeConstants.INTAKE_KV);

        SmartDashboard.putNumber("intakePullRPM", IntakeConstants.INTAKE_PULL_RPM);
        SmartDashboard.putNumber("intakeOutput", 0);


        SmartDashboard.putNumber("shootRightVoltage", ShooterConstants.SHOOTER_RIGHT_VOLTS);
        SmartDashboard.putNumber("shootLeftVoltage", ShooterConstants.SHOOTER_LEFT_VOLTS);
        SmartDashboard.putNumber("shootRightOutput", ShooterConstants.SHOOTER_RIGHT_VOLTS);
        SmartDashboard.putNumber("shootLeftOutput", ShooterConstants.SHOOTER_LEFT_VOLTS);



        SmartDashboard.putNumber("rightBumper", 0);
        SmartDashboard.putNumber("leftBumper", 0);
        SmartDashboard.putNumber("rightTrigger", 0);
        SmartDashboard.putNumber("leftTrigger", 0);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands,
        // removing finished or interrupted commands, and running subsystem
        // periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        RobotContainer.m_feederSubsystem.periodic();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link
     * RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // RobotContainer.m_intakeSubsystem.periodic();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
