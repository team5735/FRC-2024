// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightConstants;
import frc.robot.libraries.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.LimelightSubsystem;

/** An example command that uses an example subsystem. */
public class LimelightAimCommandNew extends Command {
    private LimelightSubsystem m_subsystem;
    private boolean m_targetAcquired = false;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public LimelightAimCommandNew(final LimelightSubsystem subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        m_subsystem = subsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        LimelightTarget_Fiducial[] targets = m_subsystem.getTargetedFiducials();
        if (targets.length == 0) {
            return;
        }
        m_targetAcquired = true;

        // TODO: determine unknown
        // coordinate system: x along long side with positive towards red alliance, y
        // along short side with positive facing opposite the side that theta zero
        // faces, z up, positive theta is counterclockwise and theta 0 is facing the red
        // alliance speaker.
        Optional<Alliance> ally = DriverStation.getAlliance();
        Alliance alliance = ally.isPresent() ? ally.get() : Alliance.Red;

        Pose3d currentRobotPos = targets[0].getRobotPose_FieldSpace();
        Translation3d hoodPos;
        if (alliance == Alliance.Red) {
            hoodPos = LimelightConstants.HOOD_POS;
        } else {
            hoodPos = LimelightConstants.HOOD_POS.unaryMinus();
        }
        if (!checkBotCanAim(currentRobotPos.getTranslation(), hoodPos)) {
            SmartDashboard.putNumber("cantAimDist", currentRobotPos.getTranslation().toTranslation2d().getDistance(
                    hoodPos.toTranslation2d()));
            return;
        }

        Translation2d robotToHood = hoodPos.minus(currentRobotPos.getTranslation()).toTranslation2d();
        // mathing is TODO TODO TODO
        double robotDesiredAngleRads = Math.atan2(robotToHood.getY(), robotToHood.getX());

        SmartDashboard.putNumber("aimcmdv2_desAngRads", robotDesiredAngleRads);
        SmartDashboard.putNumber("aimcmdv2_desAngDegs", Math.toDegrees(robotDesiredAngleRads));
        SmartDashboard.putNumber("aimcmdv2_robDst", robotToHood.getNorm());
    }

    // checks to see if the robot is within reasonable shooting range of the target
    private boolean checkBotCanAim(Translation3d bot, Translation3d target) {
        boolean botIsCloseEnough = bot.toTranslation2d()
                .getDistance(target.toTranslation2d()) < LimelightConstants.BOT_SHOOTING_DISTANCE;
        // TODO: do a quadratic equation intsead.
        // Translation2d targetToBot = target.minus(bot).toTranslation2d();
        return botIsCloseEnough;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
