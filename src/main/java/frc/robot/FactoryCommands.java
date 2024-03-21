package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.factories.RunOnceUntilCommand;
import frc.robot.factories.RunUntilCommand;

public class FactoryCommands {
    public static Command runUntil(Runnable action, Supplier<Boolean> isFinished, Subsystem... requirements) {
        return new RunUntilCommand(action, isFinished, requirements);
    }

    public static Command runOnceUntil(Runnable action, Supplier<Boolean> isFinished, Subsystem... requirements) {
        return new RunOnceUntilCommand(action, isFinished, requirements);
    }
}
