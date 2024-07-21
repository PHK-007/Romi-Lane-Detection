// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.sensors.IMUReader;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {
    private static final double kCountsPerRevolution = 1440.0;
    private static final double kWheelDiameterInch = 2.75591; // 70 mm
    private static final double kWheelDiameterCM = 7.0;
    
    
    public enum TurnDirection {
        CLOCKWISE, COUNTER_CLOCKWISE, IDLE;
    }
    
    private TurnDirection currTurnDir = TurnDirection.IDLE;
    
    private double currentBearing = 0.0;
    private double targetBearing = 0.0;
    private double error = currentBearing - targetBearing;
    private double netError = error;
    public final double BEARING_TOLERANCE = 0.36;
    
    
    private static Drivetrain instance = null;
    
    public static Drivetrain getInstance() {
        instance = instance == null ? new Drivetrain() : instance;
        return instance;
    }
    
    
    // The Romi has the left and right motors set to
    // PWM channels 0 and 1 respectively
    private final Spark m_leftMotor = new Spark(0);
    private final Spark m_rightMotor = new Spark(1);
    
    private PIDController leftPID = new PIDController(0.04, 0.4, 0);
    private PIDController rightPID = new PIDController(0.04, 0.4, 0);
    
    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder m_leftEncoder = new Encoder(4, 5);
    private final Encoder m_rightEncoder = new Encoder(6, 7);
    
    // Set up the differential drive controller
    private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    
    // Set up the RomiGyro
    private final RomiGyro m_gyro = new RomiGyro();
    
    // Set up the BuiltInAccelerometer
    private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
    
    /** Creates a new Drivetrain. */
    public Drivetrain() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightMotor.setInverted(true);
        
        // Use centimeters as unit for encoder distances
        m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterCM) / kCountsPerRevolution);
        m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterCM) / kCountsPerRevolution);
        resetEncoders();
    }
    
    public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
        m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    }
    
    public void voltageDrive(double leftVoltage, double rightVoltage) {
        m_diffDrive.voltageDrive(leftVoltage, rightVoltage);
    }
    
    public double uL;
    public double uR;
    /**
    * Set voltage to left and right motors using PID speed control
    * @param leftSpeed left target speed in m/s
    * @param rightSpeed right target speed in m/s
    */
    public void velocityPIDDrive(double leftSpeed, double rightSpeed) {
        uL = leftPID.getTargetVoltage(leftSpeed, getLeftSpeed());
        uR = rightPID.getTargetVoltage(rightSpeed, getRightSpeed());
        m_diffDrive.voltageDrive(uL, uR);
        // SmartDashboard.putNumber("Left Voltage", uL);
        // SmartDashboard.putNumber("Right Voltage", uR);
    }
    
    
    public void resetPIDControllers() {
        leftPID.reset();
        rightPID.reset();
    }
    
    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }
    
    public int getLeftEncoderCount() {
        return m_leftEncoder.get();
    }
    
    public int getRightEncoderCount() {
        return m_rightEncoder.get();
    }
    
    
    public double getLeftDistanceInch() {
        return m_leftEncoder.getDistance() / 2.54;
    }
    
    public double getRightDistanceInch() {
        return m_rightEncoder.getDistance() / 2.54;
    }
    
    
    public double getLeftDistanceCM() {
        return m_leftEncoder.getDistance();
    }
    
    public double getRightDistanceCM() {
        return m_rightEncoder.getDistance();
    }
    
    
    public double getAverageDistanceInch() {
        return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
    }
    
    
    public double getAverageDistanceCM() {
        return (getLeftDistanceCM() + getRightDistanceCM()) / 2.0;
    }
    
    /**
    * Get the current speed of the left motor
    * 
    * @return The current speed in cm/s
    */
    public double getLeftSpeed() {
        // Returns speed in cm/s
        return m_leftEncoder.getRate();
    }
    
    /**
    * Get the current speed of the right motor
    * 
    * @return The current speed in cm/s
    */
    public double getRightSpeed() {
        // Returns speed in cm/s
        return m_rightEncoder.getRate();
    }
    
    
    // Motor Controls
    
    public void setLeftMotorVoltage (double voltage) {
        m_leftMotor.setVoltage(voltage);
    }
    
    public void setRightMotorVoltage (double voltage) {
        m_rightMotor.setVoltage(voltage);
    }
    
    /**
    * Apply power to the left motor in percent output
    * @param power input power in the range [-1.0, 1.0]
    */
    public void setLeftMotorPower (double power) {
        m_leftMotor.set(power);
    }
    
    /**
    * Apply power to the right motor in percent output
    * @param power input power in the range [-1.0, 1.0]
    */
    public void setRightMotorPower (double power) {
        m_rightMotor.set(power);
    }
    
    
    /**
    * The acceleration in the X-axis.
    *
    * @return The acceleration of the Romi along the X-axis in Gs
    */
    public double getAccelX() {
        return m_accelerometer.getX();
    }
    
    /**
    * The acceleration in the Y-axis.
    *
    * @return The acceleration of the Romi along the Y-axis in Gs
    */
    public double getAccelY() {
        return m_accelerometer.getY();
    }
    
    /**
    * The acceleration in the Z-axis.
    *
    * @return The acceleration of the Romi along the Z-axis in Gs
    */
    public double getAccelZ() {
        return m_accelerometer.getZ();
    }
    
    /**
    * Current angle of the Romi around the X-axis.
    *
    * @return The current angle of the Romi in degrees
    */
    public double getGyroAngleX() {
        return m_gyro.getAngleX();
    }
    
    /**
    * Current angle of the Romi around the Y-axis.
    *
    * @return The current angle of the Romi in degrees
    */
    public double getGyroAngleY() {
        return m_gyro.getAngleY();
    }
    
    /**
    * Current angle of the Romi around the Z-axis.
    *
    * @return The current angle of the Romi in degrees
    */
    public double getGyroAngleZ() {
        return m_gyro.getAngleZ();
    }
    
    public double getGyroRateX() {
        return m_gyro.getRateX();
    }
    
    public double getGyroRateY() {
        return m_gyro.getRateY();
    }
    
    public double getGyroRateZ() {
        return m_gyro.getRateZ();
    }
    
    /** Reset the gyro. */
    public void resetGyro() {
        m_gyro.reset();
    }
    
    public void updateCurrentBearing() {
        currentBearing = IMUReader.getInstance().getyaw();
    }
    
    public double getCurrentBearing() {
        return currentBearing;
    }
    
    public double getTargetBearing() {
        return targetBearing;
    }
    
    public void setTargetBearing(double target) {
        targetBearing = target;
    }
    
    public void changeTargetBearing(double change) {
        targetBearing += change;
        targetBearing %= 360;
    }
    
    public double getNetGyroError() {
        return netError;
    }
    
    public TurnDirection getCurrentTurnDirection() {
        return currTurnDir;
    }
    
    public void updateTurnDirection() {
        // Update error
        error = targetBearing - currentBearing;
        
        if (error < -180) {
            netError = error + 360;
        } else if (error < 0) {
            netError = Math.abs(error);
        } else if (error > 180) {
            netError = error - 180;
        } else if (error > 0) {
            netError = error;
        }
        
        if (netError <= BEARING_TOLERANCE) {
            currTurnDir = TurnDirection.IDLE;
        } else {
            if (error < -180) {
                currTurnDir = TurnDirection.CLOCKWISE;
            } else if (error < 0) {
                currTurnDir = TurnDirection.COUNTER_CLOCKWISE;
            } else if (error > 180) {
                currTurnDir = TurnDirection.COUNTER_CLOCKWISE;
            } else if (error > 0) {
                currTurnDir = TurnDirection.CLOCKWISE;
            }
        }
    }
    
    
    public double calculateTurnSpeed (TurnDirection dir, double error) {
        double kP = 0.0085;
        double turnDirection;
        if (dir == TurnDirection.CLOCKWISE) {
            turnDirection = -1;
        } else if (dir == TurnDirection.COUNTER_CLOCKWISE) {
            turnDirection = 1;
        } else {
            turnDirection = 0;
        }
        return (kP * Math.abs(error)) * turnDirection;
    }
    
    
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateCurrentBearing();
        updateTurnDirection();
    }
}
