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

/**
 * This class represents a CANdleSubsystem. It utilizes a {@link CANdle} to control a series of LEDs.
 */
public class CANdleSubsystem extends SubsystemBase {
    private CANdle candle;

    /** Creates a new CANdleSubsystem. */
    public CANdleSubsystem() {
        candle = new CANdle(Constants.CANDLE_ID);
    }

    /**
     * Sets the {@link CANdle} to a present idle animation, this being a rainbow.
     */
    public void setIdle() {
        candle.animate(new RainbowAnimation(0.5, 1.0, 8));
    }

    /**
     * @return a Command to set the {@link CANdle} to a predetermined ready color, as outlined in {@link CANdleConstants}.
     */
    public Command colorReady() {
        return setToColorByID(CANdleConstants.READY);
    }

    /**
     * @return a Command to set the {@link CANdle} to a predetermined auto color, as outlined in {@link CANdleConstants}.
     */
    public Command colorAuto() {
        return setToColorByID(CANdleConstants.AUTO);
    }
    
    /**
     * @return a Command to set the {@link CANdle} to a predetermined aiming color, as outlined in {@link CANdleConstants}.
     */
    public Command colorAiming() {
        return setToColorByID(CANdleConstants.AIMING);
    }

    /**
     * @return a Command to set the {@link CANdle} to a predetermined aimed color, as outlined in {@link CANdleConstants}.
     */
    public Command colorAimed() {
        return setToColorByID(CANdleConstants.AIMED);
    }

    /**
     * @return a Command to set the {@link CANdle} to a predetermined color for running intake, as outlined in {@link CANdleConstants}.
     */
    public Command colorIntakeRunning() {
        return setToColorByID(CANdleConstants.INTAKE_RUNNING);
    }

    /**
     * @return a Command to set the {@link CANdle} to a predetermined shooting color, as outlined in {@link CANdleConstants}.
     */
    public Command colorShooting() {
        return setToColorByID(CANdleConstants.SHOOTING);
    }

    /**
     * Creates a {@code runOnce} {@link Command} to set the leds to a certain color.
     * 
     * @param id - the index of the selected {@link Color} in
     *           {@link CANdleConstants}'s {@code COLORS} array.
     * @return a {@link Command} which will requre this subsytem, run the
     *         {@code setToColor} method with the selected {@link Color} and
     *         promptly end.
     */
    private Command setToColorByID(int id) {
        return runOnce(() -> {
            setToColor(CANdleConstants.COLORS[id]);
        });
    }

    /**
     * Method to set the {@link CANdle}'s LEDs to a passed-in {@link Color} value.
     * 
     * @param color - the {@link Color} to set the {@link CANdle}'s LEDs to.
     */
    private void setToColor(Color color) {
        candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    }
}
