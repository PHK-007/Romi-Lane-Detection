package frc.robot.sensors;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    
    private NetworkTable laneTable;
    private DoubleArrayEntry x1Entry, x2Entry;
    private NetworkTableInstance inst;
    private DoubleArrayTopic x1Topic, x2Topic;

    private double[] x1, x2;
    private double x1Mid, x2Mid;
    public final double MIDPOINT = 320.0;
    private final double[] DEFAULT_POSITION = {-1, -1};


    private static Vision instance = null;
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public Vision() {
        inst = NetworkTableInstance.getDefault();
        laneTable = inst.getTable("LaneDet");
        x1Topic = laneTable.getDoubleArrayTopic("X1 Topic");
        x2Topic = laneTable.getDoubleArrayTopic("X2 Topic");
        
        x1Entry = x1Topic.getEntry(DEFAULT_POSITION);
        x2Entry = x2Topic.getEntry(DEFAULT_POSITION);

        x1 = new double[2];
        x2 = new double[2];
    }


    public void update() {
        x1 = x1Entry.get();
        x2 = x2Entry.get();

        x1Mid = (x1[0] + x1[1]) / 2;
        x2Mid = (x2[0] + x2[1]) / 2;
    }


    public double[] getLeftLaneEdges() {
        return x1;
    }

    public double getLeftLaneCenter() {
        return x1Mid;
    }

    public double[] getRightLaneEdges() {
        return x2;
    }

    public double getRightLaneCenter() {
        return x2Mid;
    }
}
