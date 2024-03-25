// idea stolen with <3 from team 190's 2024 code.

package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LoggedTunableNumber {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("tunables");

    private DoubleSubscriber m_subscriber;

    public LoggedTunableNumber(String name) {
        init(name, 0);
    }

    public LoggedTunableNumber(String name, double initVal) {
        init(name, initVal);
    }

    private void init(String name, double value) {
        DoubleTopic topic = table.getDoubleTopic(name);
        topic.publish().set(value);
        m_subscriber = topic.subscribe(value);
    }

    public double get() {
        return m_subscriber.get();
    }
}
