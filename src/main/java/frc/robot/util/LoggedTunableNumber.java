// idea stolen with <3 from team 190's 2024 code.

package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LoggedTunableNumber {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("tunables");

    private final DoubleSubscriber m_subscriber;

    private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    public LoggedTunableNumber(String name) {
        DoubleTopic topic = table.getDoubleTopic(name);
        m_subscriber = topic.subscribe(0);
    }

    public double get() {
        return m_subscriber.get();
    }

    /**
     * hasChanged returns whether the value has changed since the last call with the
     * same id. The recommended approach is to call hashCode() and pass that as the
     * id.
     */
    public boolean hasChanged(int id) {
        return lastHasChangedValues.get(id) == m_subscriber.get();
    }
}
