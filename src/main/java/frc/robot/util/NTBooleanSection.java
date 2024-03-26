package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTBooleanSection {
    private static final String sectionName = "sections";

    private final NetworkTable table;
    private Map<String, BooleanTopic> entries = new HashMap<>();

    /**
     * Creates a new NTBooleanSection. This makes the section as a subtable in the
     * top-level table specified by sectionName.
     *
     * @param name The name of the subtable where entries will be put.
     */
    public NTBooleanSection(String name) {
        table = NetworkTableInstance.getDefault().getTable(sectionName).getSubTable(name);
    }

    /**
     * Creates a new NTBooleanSection. This makes the section as a subtable in the
     * top-level table specified by sectionName. It also makes the specified entries
     * with addEntry.
     *
     * @param name    The name of the subtable where entries will be put.
     * @param entries The entries to create, as a shorthand.
     */
    public NTBooleanSection(String name, String... entries) {
        table = NetworkTableInstance.getDefault().getTable(sectionName).getSubTable(name);
        for (String entry : entries) {
            addEntry(entry);
        }
    }

    /**
     * Creates a new entry and adds it to the list of entries. The entry will be
     * created as a {@link BooleanTopic} with the name name in the table this
     * instance is for.
     *
     * @param name The name of the entry.
     */
    public void addEntry(String name) {
        entries.put(name, table.getBooleanTopic(name));
    }

    /**
     * Sets the entry to the value. This makes a {@link BooleanPublisher} out of the
     * entry and uses .set() to set its value in NetworkTables.
     * 
     * @param entry The entry to set
     * @param value The value to set the entry to
     */
    public void set(String entry, boolean value) {
        entries.get(entry).publish().set(value);
    }
}
