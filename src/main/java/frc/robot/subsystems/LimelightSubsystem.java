package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
    private static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    private DoubleArraySubscriber botposeSubscriber = limelightTable.getDoubleArrayTopic("botpose")
            .subscribe(new double[8]);
    private DoubleSubscriber targetVisibleSubscriber = limelightTable.getDoubleTopic("tv").subscribe(0);
    private static DoublePublisher ledModePublisher = limelightTable.getDoubleTopic("ledMode").publish();

    public Pose3d getBotPose() {
        double[] botpose = botposeSubscriber.get();
        return new Pose3d(
                botpose[0],
                botpose[1],
                botpose[2],
                new Rotation3d(botpose[3],
                        botpose[4],
                        botpose[5]));
    }

    public boolean hasTarget() {
        return targetVisibleSubscriber.get() == 1;
    }

    public int getNumTargets() {
        return (int) botposeSubscriber.get()[7];
    }

    public static void ledsOn() {
        ledModePublisher.set(3);
    }

    public static void ledsOff() {
        ledModePublisher.set(1);
    }

    public static Command blinkLedsOnce() {
        return Commands.sequence(
                Commands.runOnce(() -> ledsOn()),
                Commands.runOnce(() -> Commands.waitSeconds(LimelightConstants.BLINK_TIME)),
                Commands.runOnce(() -> ledsOff()),
                Commands.runOnce(() -> Commands.waitSeconds(LimelightConstants.BLINK_TIME)));
    }

    public static Command blinkLeds(int numberOfTimes) {
        Command currentlyBuiltCommand = Commands.none();
        for (int i = 0; i < numberOfTimes; i++) {
            currentlyBuiltCommand = currentlyBuiltCommand.andThen(blinkLedsOnce());
        }
        return currentlyBuiltCommand.andThen(Commands.runOnce(() -> ledsOff()));
    }
}
