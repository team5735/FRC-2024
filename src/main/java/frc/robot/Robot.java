// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AngleConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LimelightConstants;
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

    private PowerDistribution m_PD = new PowerDistribution(Constants.PDH_ID, ModuleType.kRev);

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button
        // bindings, and put our autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        // Put initial values of numbers that are continuously updated from
        // SmartDashboard.
        SmartDashboard.putNumber("angleKP", AngleConstants.ANGLE_KP);
        SmartDashboard.putNumber("angleKI", AngleConstants.ANGLE_KI);
        SmartDashboard.putNumber("angleKD", AngleConstants.ANGLE_KD);

        SmartDashboard.putNumber("angleKS", AngleConstants.ANGLE_KS);
        SmartDashboard.putNumber("angleKG", AngleConstants.ANGLE_KG);
        SmartDashboard.putNumber("angleKV", AngleConstants.ANGLE_KV);

        SmartDashboard.putNumber("anglePos", 0);

        SmartDashboard.putNumber("angleCurrentSetpoint", AngleConstants.ANGLE_START_POS_DEG);
        SmartDashboard.putNumber("angleNewSetpoint", AngleConstants.ANGLE_START_POS_DEG);

        SmartDashboard.putNumber("climbRightUpVoltage", ClimberConstants.CLIMBER_RIGHT_UP_VOLTS);
        SmartDashboard.putNumber("climbLeftUpVoltage", ClimberConstants.CLIMBER_LEFT_UP_VOLTS);
        SmartDashboard.putNumber("climbRightDownVoltage", ClimberConstants.CLIMBER_RIGHT_DOWN_VOLTS);
        SmartDashboard.putNumber("climbLeftDownVoltage", ClimberConstants.CLIMBER_LEFT_DOWN_VOLTS);

        SmartDashboard.putNumber("climbRightPos", 0);
        SmartDashboard.putNumber("climbLeftPos", 0);

        SmartDashboard.putNumber("climbRightOutput", 0);
        SmartDashboard.putNumber("climbLeftOutput", 0);

        SmartDashboard.putNumber("feederPullVoltage", FeederConstants.FEEDER_PULL_VOLTS);
        SmartDashboard.putNumber("feederPushVoltage", FeederConstants.FEEDER_PUSH_VOLTS);

        SmartDashboard.putBoolean("feederSwitchStatus", false);

        SmartDashboard.putNumber("intakePullVoltage", IntakeConstants.INTAKE_PULL_VOLTS);
        SmartDashboard.putNumber("intakePushVoltage", IntakeConstants.INTAKE_PUSH_VOLTS);

        SmartDashboard.putNumber("shootTopKP", ShooterConstants.SHOOTER_TOP_KP);
        SmartDashboard.putNumber("shootTopKI", ShooterConstants.SHOOTER_TOP_KI);
        SmartDashboard.putNumber("shootTopKD", ShooterConstants.SHOOTER_TOP_KD);

        SmartDashboard.putNumber("shootTopKS", ShooterConstants.SHOOTER_TOP_KS);
        SmartDashboard.putNumber("shootTopKV", ShooterConstants.SHOOTER_TOP_KV);

        SmartDashboard.putNumber("shootBottomKP", ShooterConstants.SHOOTER_BOTTOM_KP);
        SmartDashboard.putNumber("shootBottomKI", ShooterConstants.SHOOTER_BOTTOM_KI);
        SmartDashboard.putNumber("shootBottomKD", ShooterConstants.SHOOTER_BOTTOM_KD);

        SmartDashboard.putNumber("shootBottomKS", ShooterConstants.SHOOTER_BOTTOM_KS);
        SmartDashboard.putNumber("shootBottomKV", ShooterConstants.SHOOTER_BOTTOM_KV);

        SmartDashboard.putNumber("shootTopRPM", ShooterConstants.SHOOTER_TOP_DEFAULT_RPM);
        SmartDashboard.putNumber("shootBottomRPM", ShooterConstants.SHOOTER_BOTTOM_DEFAULT_RPM);
        SmartDashboard.putNumber("shootTopOutput", 0);
        SmartDashboard.putNumber("shootBottomOutput", 0);

        SmartDashboard.putNumber("drivetrain_slowSpeed", DrivetrainConstants.SLOW_SPEED);
        SmartDashboard.putNumber("drivetrain_normalSpeed", DrivetrainConstants.NORMAL_SPEED);
        SmartDashboard.putNumber("drivetrain_turboSpeed", DrivetrainConstants.TURBO_SPEED);

        SmartDashboard.putNumber("llv2_turnP", LimelightConstants.TURN_P);
        SmartDashboard.putNumber("llv2_turnI", LimelightConstants.TURN_I);
        SmartDashboard.putNumber("llv2_turnD", LimelightConstants.TURN_D);

        SmartDashboard.putNumber("testShootAngle", AngleConstants.ANGLE_START_POS_DEG);
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

        SmartDashboard.putNumber("PD 6", m_PD.getCurrent(6));
        SmartDashboard.putNumber("PD total", m_PD.getTotalCurrent());
        SmartDashboard.putNumber("PD total voltage", m_PD.getVoltage());
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
        m_robotContainer.resetShenanigans();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
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
