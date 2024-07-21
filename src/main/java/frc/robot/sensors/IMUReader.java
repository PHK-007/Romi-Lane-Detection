package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class IMUReader {

    private NetworkTable imuTable;
    private NetworkTableEntry pitchEntry, yawEntry, rollEntry;

    NetworkTableInstance inst;


    private double pitch, yaw, roll;
    private double pitchOffset, yawOffset ,rollOffset;

    private static IMUReader instance = null;
    public static IMUReader getInstance() {
        if (instance == null) {
            instance = new IMUReader();
        }
        return instance;
    }

    public IMUReader() {
        inst = NetworkTableInstance.getDefault();
        inst.startClient3("125");
        inst.startDSClient();
        inst.setServer("localhost", NetworkTableInstance.kDefaultPort3);

        imuTable = inst.getTable("9DoF");
        pitchEntry = imuTable.getEntry("pitch");
        yawEntry = imuTable.getEntry("yaw");
        rollEntry = imuTable.getEntry("roll");

        pitchOffset = 0;
        yawOffset = 0;
        rollOffset = 0;
    }

    public void updateAngles() {
        pitch = pitchEntry.getDouble(0.0) + pitchOffset;
        yaw = -yawEntry.getDouble(0.0) + yawOffset;
        while (yaw < 0) yaw += 360;
        while (yaw > 360) yaw -= 360;
        roll = rollEntry.getDouble(0.0) + rollOffset;
    }

    public double getPitch() {
        return pitch;
    }

    public double getyaw() {
        return yaw;
    }

    public double getRoll() {
        return roll;
    }

    public void reset() {
        pitchOffset += 0 - pitch;
        yawOffset += 0 - yaw;
        rollOffset += 0 - roll;
        updateAngles();
    }
}