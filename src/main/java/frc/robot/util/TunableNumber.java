// idea stolen with <3 from team 190's 2024 code.

package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TunableNumber {
    private DoubleSubscriber m_subscriber;

    public TunableNumber(String name) {
        init("", name, 0);
    }

    public TunableNumber(String name, double initVal) {
        init("", name, initVal);
    }

    public TunableNumber(String subtable, String name) {
        init(subtable, name, 0);
    }

    public TunableNumber(String subtable, String name, double initVal) {
        init(subtable, name, initVal);
    }

    private void init(String subtable, String name, double value) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("tunables");
        if (subtable != "") {
            table = table.getSubTable(subtable);
        }
        DoubleTopic topic = table.getDoubleTopic(name);
        topic.publish().set(value);
        m_subscriber = topic.subscribe(value);
    }

    public double get() {
        return m_subscriber.get();
    }
}
