// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// this file is a massive TODO

package frc.robot.subsystems;

import java.awt.Color;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANdleConstants;
import frc.robot.constants.Constants;

public class CANdleSubsystem extends SubsystemBase {
    private CANdle candle;

    /** Creates a new CANdle Subsystem. */
    public CANdleSubsystem() {
        candle = new CANdle(Constants.CANDLE_ID);
    }

    public void setIdle() {
        candle.animate(new RainbowAnimation(0.5, 1.0, 8));
    }

    public Command colorAiming() {
        return runOnce(() -> {
            setToColor(CANdleConstants.COLORS[CANdleConstants.AIMING]);
        });
    }

    public Command colorAimed() {
        return runOnce(() -> {
            setToColor(CANdleConstants.COLORS[CANdleConstants.AIMED]);
        });
    }

    public Command colorReady() {
        return runOnce(() -> {
            setToColor(CANdleConstants.COLORS[CANdleConstants.READY]);
        });
    }

    public Command colorIntakeRunning() {
        return runOnce(() -> {
            setToColor(CANdleConstants.COLORS[CANdleConstants.INTAKE_RUNNING]);
        });
    }

    private void setToColor(Color color) {
        setDefaultSettings();
        candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    }

    private void setDefaultSettings() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        candle.configAllSettings(config);
    }

    /**
     * An example method querying a boolean state of the subsystem (for example,
     * a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
