package frc.robot.sensors;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    
    private NetworkTable laneTable;
    private DoubleArraySubscriber x1sub, x2sub;
    private NetworkTableInstance inst;

    private double[] x1, x2;
    private double x1Mid = -1;
    private double x2Mid = -1;
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

        x1sub = laneTable.getDoubleArrayTopic("x1").subscribe(DEFAULT_POSITION);
        x2sub = laneTable.getDoubleArrayTopic("x2").subscribe(DEFAULT_POSITION);

        x1 = new double[2];
        x2 = new double[2];
    }


    public void update() {
        x1 = x1sub.get();
        x2 = x2sub.get();

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
