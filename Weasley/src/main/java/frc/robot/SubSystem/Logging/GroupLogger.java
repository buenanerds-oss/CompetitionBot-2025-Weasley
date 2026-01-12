package frc.robot.SubSystem.Logging;

import java.util.HashMap;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class GroupLogger{
    private static HashMap<String, double[]> doubleGroups = new HashMap<>();
    private static HashMap<String, StructSerializable[]> StructGroups = new HashMap<>();

    /**
     * logs multiple variables, but only updates logs when it reaches the expected capacity
     * used in cases where things need to be put together before they can be logged together (like the modules of swerveDrive)
     * @param name
     * @param value
     * @param index
     * @param capacity - max amount your putting into the logs, starts count from 1
     */
    public static void logDoubleGroup(String name, double value, int index, int capacity) {
        if (doubleGroups.containsKey(name)) doubleGroups.put(name, new double[capacity]);
        doubleGroups.get(name)[index] = value;
        if (index == capacity - 1) NerdLog.logDoubleArray(name, doubleGroups.get(name));
    }

    public static <T extends StructSerializable> void logStructGroup(String name, T value, Struct valueStruct, int index, int capacity) {
        if (StructGroups.containsKey(name)) StructGroups.put(name, new StructSerializable[capacity]);
        StructGroups.get(name)[index] = value;
        if (index == capacity - 1)NerdLog.LogStructArray(name, StructGroups.get(name), valueStruct);
    }
}
