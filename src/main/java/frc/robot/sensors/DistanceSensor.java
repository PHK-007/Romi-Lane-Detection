package frc.robot.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogInput;

public class DistanceSensor extends SubsystemBase{

    private AnalogInput sensor;
    private boolean start = false;
    private boolean duringAuton = false;

    static DistanceSensor instance = null;
    public static DistanceSensor getInstance() {
        instance = instance == null ? new DistanceSensor() : instance;
        return instance;
    }
    
    public DistanceSensor() {
        // sensor = new AnalogInput(1);
        sensor = new AnalogInput(2);
    }

    public double getVoltage() {
        return sensor.getVoltage();
    }

    public void setDuringAuton(boolean condition) {
        duringAuton = condition;
    }

    public boolean startAuton() {
        if ((sensor.getVoltage() >= 1.5) && duringAuton) {
            start = true;
        } else {
            start = false;
        }
        return start;
    }

    public void setStart(boolean value) {
        start = value;
    }
}
