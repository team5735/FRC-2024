// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.Color;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANdleConstants;
import frc.robot.constants.Constants;

public class CANdleSubsystem extends SubsystemBase {
    private CANdle candle;

    /** Creates a new CANdleSubsystem. */
    public CANdleSubsystem() {
        candle = new CANdle(Constants.CANDLE_ID);
    }

    public void setIdle() {
        candle.animate(new RainbowAnimation(0.5, 1.0, 8));
    }

    public Command colorReady() {
        return setToColorByID(CANdleConstants.READY);
    }

    public Command colorAuto() {
        return setToColorByID(CANdleConstants.AUTO);
    }

    public Command colorAiming() {
        return setToColorByID(CANdleConstants.AIMING);
    }

    public Command colorAimed() {
        return setToColorByID(CANdleConstants.AIMED);
    }

    public Command colorIntakeRunning() {
        return setToColorByID(CANdleConstants.INTAKE_RUNNING);
    }

    public Command colorShooting() {
        return setToColorByID(CANdleConstants.SHOOTING);
    }

    private Command setToColorByID(int id) {
        return runOnce(() -> {
            setToColor(CANdleConstants.COLORS[id]);
        });
    }

    private void setToColor(Color color) {
        candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    }
}
