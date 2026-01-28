package frc.robot.SubSystem.Logging;

import java.util.HashMap;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class NerdLog {

    static NetworkTable baseTable;
    static HashMap<String, StructPublisher> structPublishers; 
    static HashMap<String, DoublePublisher> doublePublishers;
    static HashMap<String, DoubleArrayPublisher> doubleArrayPublishers;
    static HashMap<String, StructArrayPublisher> structArrayPublishers;
    static NetworkTableInstance tableInst;


    /**
     * starts the logging and fetches es all the annotations
     */
    public static void startLog() {
        startLog("", "");
    }

    /**
     * starts the logging and fetches all the annotations
     */
    public static void startLog(String directory) {
        startLog(directory,"");
    }
    
    /**
     * starts the logging and fetches all the annotations
     */
    public static void startLog(String directory, String fileName) {
        DataLogManager.start(directory, fileName);
        DriverStation.startDataLog(DataLogManager.getLog());

        tableInst = NetworkTableInstance.getDefault();
        baseTable = tableInst.getTable("Robot");
        structPublishers = new HashMap<>();
        doublePublishers = new HashMap<>();
        doubleArrayPublishers = new HashMap<>();
        structArrayPublishers = new HashMap<>();
    }

    //uses reflection and reflection scares me...
    public static void logClassesWithAnnotation(Class... classes) {

    }


    public static <T extends StructSerializable> void logStructvariable(String name, T variable, Struct variableStruct) {
        // if the variable doesn't already exist, add it.
        if (/*!structVariables.containsKey(name) &&*/ !structPublishers.containsKey(name)) {
            StructPublisher<T> structPub = baseTable.getStructTopic(name, variableStruct).publish();
            structPublishers.put(name, structPub);
        }

        StructPublisher<T> structPub = structPublishers.get(name);
        structPub.set(variable);
    }

    public static <T extends StructSerializable> void LogStructArray(String name, T[] variables, Struct variableStruct) {
        if (!structArrayPublishers.containsKey(name)) {
            StructArrayPublisher<T> structPub = baseTable.getStructArrayTopic(name, variableStruct).publish();
            structArrayPublishers.put(name, structPub);
        }
        StructArrayPublisher structPub = structArrayPublishers.get(name);
        structPub.set(variables);
    }

    public static void logDouble(String name, double variable) {
        if (!doublePublishers.containsKey(name)) {
            DoublePublisher dubPub = baseTable.getDoubleTopic(name).publish();
            doublePublishers.put(name, dubPub);
        }

        DoublePublisher dubPub = doublePublishers.get(name);
        dubPub.set(variable);
    }

    public static void logDoubleArray(String name, double[] variable) {
        if (!doubleArrayPublishers.containsKey(name)) {
            DoubleArrayPublisher dubPub = baseTable.getDoubleArrayTopic(name).publish();
            doubleArrayPublishers.put(name, dubPub);
        }

        DoubleArrayPublisher dubPub = doubleArrayPublishers.get(name);
        dubPub.set(variable);
    }

    }
