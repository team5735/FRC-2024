package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTDoubleSection {
    private static final String sectionName = "sections";

    private final NetworkTable table;
    private Map<String, DoubleTopic> entries = new HashMap<>();

    /**
     * Creates a new NTDoubleSection. This makes the section as a subtable in the
     * top-level table specified by sectionName.
     *
     * @param name The name of the subtable where entries will be put.
     */
    public NTDoubleSection(String name) {
        table = NetworkTableInstance.getDefault().getTable(sectionName).getSubTable(name);
    }

    /**
     * Creates a new NTDoubleSection. This makes the section as a subtable in the
     * top-level table specified by sectionName. It also makes the specified entries
     * with addEntry.
     *
     * @param name    The name of the subtable where entries will be put.
     * @param entries The entries to create, as a shorthand.
     */
    public NTDoubleSection(String name, String... entries) {
        table = NetworkTableInstance.getDefault().getTable(sectionName).getSubTable(name);
        for (String entry : entries) {
            addEntry(entry);
        }
    }

    /**
     * Creates a new entry and adds it to the list of entries. The entry will be
     * created as a {@link DoubleTopic} with the name name in the table this
     * instance is for.
     *
     * @param name The name of the entry.
     */
    public void addEntry(String name) {
        entries.put(name, table.getDoubleTopic(name));
    }

    /**
     * Sets the entry to the value. This makes a {@link DoublePublisher} out of the
     * entry and uses .set() to set its value in NetworkTables.
     * 
     * @param entry The entry to set
     * @param value The value to set the entry to
     */
    public void set(String entry, double value) {
        entries.get(entry).publish().set(value);
    }

    /**
     * Gets the entry and makes a DoubleSubscriber out of it, and get its
     * result. The subscriber is created with a default value of 0.
     *
     * @param entry What entry to get
     *
     * @return The value in that entry, according to NetworkTables
     */
    public double get(String entry) {
        return get(entry, 0);
    }

    /**
     * Gets the entry and makes a DoubleSubscriber out of it, and get its
     * result. The subscriber is created with a default value of defaultVal.
     *
     * @param entry      What entry to get
     * @param defaultVal The default value for the subscriber
     *
     * @return The value in that entry, according to NetworkTables
     */
    public double get(String entry, double defaultVal) {
        return entries.get(entry).subscribe(defaultVal).get();
    }
}
