package frc.robot.constants;

import java.awt.Color;

public class CANdleConstants {
    public static final int READY = 0;
    public static final int AUTO = 1;
    public static final int AIMING = 2;
    public static final int AIMED = 3;
    public static final int INTAKE_RUNNING = 4;
    public static final int SHOOTING = 5;

    public static final Color[] COLORS = { 
        new Color(0, 0, 255), // BLUE (READY)
        new Color(127, 0, 127), // PURPLE (AUTO)
        new Color(127, 127, 0), // YELLOW-GREEN (AIMING)
        new Color(0, 255, 0),  // GREEN (AIMED)
        new Color(127, 0, 0), // DIM RED (INTAKING)
        new Color(255, 0, 0)  // RED (SHOOTING)
    };
}
