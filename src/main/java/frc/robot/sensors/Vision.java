package frc.robot.sensors;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    
    private NetworkTable laneTable;
    private DoubleArraySubscriber x1sub, x2sub;
    private NetworkTableInstance inst;

    private double[] x1, x2;
    private double x1Mid = -1;
    private double x2Mid = -1;
    private double midLine = -1;

    /**
     * The estimated lateral offset
     * <p> Positive: the robot is to the right of the expectation line
     * <p> Negative: the robot is to the left of the expectation line
     */
    private double lateralOffset = Double.NaN;
    
    public static final double SCREEN_MIDPOINT = 320.0;
    public static final int NUMBER_OF_CAMERAS = 1;
    private final double offsetFactor = 1; // TODO Tune this value
    private final double[] DEFAULT_POSITION = {-1, -1};

    private DoublePublisher testPub;

    private static Vision instance = null;
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public Vision() {
        inst = NetworkTableInstance.getDefault();
        inst.startClient4("Romi Client");
        inst.setServer("10.0.0.2", 1735);
        inst.addConnectionListener(true, (event) -> {
            SmartDashboard.putString("NT Connection event: remote ID", event.connInfo.remote_id);
            SmartDashboard.putString("NT Connection event: remote IP", event.connInfo.remote_ip);
        });
        

        laneTable = inst.getTable("LaneDet");

        x1sub = laneTable.getDoubleArrayTopic("x1").subscribe(DEFAULT_POSITION);
        x2sub = laneTable.getDoubleArrayTopic("x2").subscribe(DEFAULT_POSITION);

        x1 = new double[2];
        x2 = new double[2];

        testPub = laneTable.getDoubleTopic("Romi Side Test: myX").publish();
    }


    public void update() {
        testPub.set(getMyX());        
        x1 = x1sub.get();
        x2 = x2sub.get();

        x1Mid = (x1[0] + x1[1]) / 2;
        x2Mid = (x2[0] + x2[1]) / 2;

        if (NUMBER_OF_CAMERAS == 1) {
            midLine = x1Mid;
        } else {
            midLine = (x1Mid + x2Mid) / 2;
        }

        lateralOffset = offsetFactor * (midLine - SCREEN_MIDPOINT);
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

    public double getExpectationLine() {
        return midLine;
    }

    public double getLateralOffset() {
        return lateralOffset;
    }

    public double getMyX() {
        return laneTable.getEntry("testValue").getDouble(-1.0);
    }
}
