package frc.robot.subsystems.vision;

public interface VisionIO {
    public default void updateResults(VisionResults results) {
    }

    public default void ledsOn() {
    }

    public default void ledsOff() {
    }

    public default boolean hasTarget() {
        return false;
    }
}
