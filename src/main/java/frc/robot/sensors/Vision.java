package frc.robot.sensors;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Vision {
    
    private NetworkTable laneTable;
    private DoubleArraySubscriber x1Sub, x2Sub;
    private BooleanSubscriber detectedSub;
    private NetworkTableInstance inst;

    private double[] x1, x2;
    private double x1Mid = -1;
    private double x2Mid = -1;
    private double midLine = -1;
    private boolean laneDetected = false;

    /**
     * The estimated lateral offset
     * <p> Positive: the robot is to the right of the expectation line
     * <p> Negative: the robot is to the left of the expectation line
     */
    private double lateralOffset = Double.NaN;
    
    public static final double SCREEN_MIDPOINT = 320.0;
    public static final int NUMBER_OF_LANES = 1;
    private final double offsetFactor = 1; // TODO Tune this value
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
        inst.startServer();

        laneTable = inst.getTable("LaneDet");

        x1Sub = laneTable.getDoubleArrayTopic("x1").subscribe(DEFAULT_POSITION);
        x2Sub = laneTable.getDoubleArrayTopic("x2").subscribe(DEFAULT_POSITION);
        detectedSub = laneTable.getBooleanTopic("Lane Detected").subscribe(false);

        x1 = new double[2];
        x2 = new double[2];
        laneDetected = false;
    }


    public void update() {  
        updateVariables();

        x1Mid = (x1[0] + x1[1]) / 2;
        x2Mid = (x2[0] + x2[1]) / 2;

        if (NUMBER_OF_LANES == 1) {
            midLine = x1Mid;
        } else {
            midLine = (x1Mid + x2Mid) / 2;
        }

        lateralOffset = offsetFactor * (midLine - SCREEN_MIDPOINT);
    }

    private void updateVariables() {
        x1 = x1Sub.get();
        x2 = x2Sub.get();
        laneDetected = detectedSub.get();
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

    public boolean isLaneDetected() {
        return laneDetected;
    }
}
