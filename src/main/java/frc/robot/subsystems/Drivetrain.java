// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import frc.robot.sensors.Vision;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {
    private static final double kCountsPerRevolution = 1440.0;
    private static final double kWheelDiameterInch = 2.75591; // 70 mm
    private static final double kWheelDiameterCM = 7.0;
    
    
    public enum TurnDirection {
        CLOCKWISE, COUNTER_CLOCKWISE, IDLE;
    }
    
    private TurnDirection currTurnDir = TurnDirection.IDLE;
    
    private double error = 0;
    public final double ERROR_TOLERANCE = 2;
    
    
    private static Drivetrain instance = null;
    public static Drivetrain getInstance() {
        instance = instance == null ? new Drivetrain() : instance;
        return instance;
    }
    

    private double uL = 0.0;
    private double uR = 0.0;

    
    // The Romi has the left and right motors set to
    // PWM channels 0 and 1 respectively
    private final Spark m_leftMotor = new Spark(0);
    private final Spark m_rightMotor = new Spark(1);
    
    private PIDController leftPID = new PIDController(0.125, 0.0, 0);
    private PIDController rightPID = new PIDController(0.125, 0.0, 0);
    
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

    public void arcadeDrive(double xaxisSpeed, double zaxisRotate, boolean squareInputs) {
        m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate, squareInputs);
    }
    
    public void voltageDrive(double leftVoltage, double rightVoltage) {
        m_diffDrive.voltageDrive(leftVoltage, rightVoltage);
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
    
    public double getAverageDistanceInch() {
        return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
    }
    
    public double getLeftDistanceCM() {
        return m_leftEncoder.getDistance();
    }
    
    public double getRightDistanceCM() {
        return m_rightEncoder.getDistance();
    }
    
    public double getAverageDistanceCM() {
        return (getLeftDistanceCM() + getRightDistanceCM()) / 2.0;
    }
    

    public Command setDriveVoltage(double voltage) {
        return Commands.parallel(setLeftVoltageCommand(voltage), setRightVoltageCommand(voltage));
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

    /**
     * Sets the variable uL to the specified voltage
     * @param voltage
     */
    public Command setLeftVoltageCommand(double voltage) {
        return Commands.runOnce(() -> uL = voltage);
    }

    /**
     * 
     * @return The left voltage output from the PID controller: uL
     */
    public double getLeftVoltage() {
        return uL;
    }


    /**
     * Sets the variable uR to the specified voltage
     * @param voltage
     */
    public Command setRightVoltageCommand(double voltage) {
        return Commands.runOnce(() -> uR = voltage);
    }

    /**
     * 
     * @return The right voltage output from the PID controller: uR 
     */
    public double getRightVoltage() {
        return uR;
    }

    public Command setVelocityCommand(double vel) {
        return setVelocityCommand(vel, vel);
    }


    public Command setVelocityCommand(double left, double right) {
        Command seq = Commands.runOnce(() -> {
            leftPID.setSetpoint(left);
            rightPID.setSetpoint(right);
        }, this);
        return seq;
    }

    
    /**
     * Sets the left motor to the specified voltage
     * @param voltage
     */
    // public void setLeftMotorVoltage (double voltage) {
    //     m_leftMotor.setVoltage(voltage);
    // }
    
    /**
     * Sets the right motor to the specified voltage
     * @param voltage
     */
    // public void setRightMotorVoltage (double voltage) {
    //     m_rightMotor.setVoltage(voltage);
    // }
    
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
    
    public TurnDirection getCurrentTurnDirection() {
        return currTurnDir;
    }
    
    public void updateTurnDirection() {
        // Update error
        error = Vision.getInstance().getError();
        
        if (Math.abs(error) <= ERROR_TOLERANCE) {
            currTurnDir = TurnDirection.IDLE;
        } else if (error > 0) {
            currTurnDir = TurnDirection.CLOCKWISE;
        } else {
            currTurnDir = TurnDirection.COUNTER_CLOCKWISE;
        }
    }
    
    
    /**
     * Calculates the turn speed using the currTurnDir and error from Drivetrain.java
     * @return The value to be used for arcadeDrive()
     */
    public double calculateTurnSpeed () {
        double kP = 0.0085;
        double turnDirection;
        if (currTurnDir == TurnDirection.CLOCKWISE) {
            turnDirection = -1;
        } else if (currTurnDir == TurnDirection.COUNTER_CLOCKWISE) {
            turnDirection = 1;
        } else {
            turnDirection = 0;
        }
        // return kP * error * -1; // This should also technically be correct but won't account for tolerance
        return (kP * Math.abs(error)) * turnDirection; 
    }

    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateTurnDirection();

        // uL = leftPID.calculate(getLeftSpeed());
        // uR = rightPID.calculate(getRightSpeed());

        // // m_leftMotor.setVoltage(uL);
        // // m_rightMotor.setVoltage(uR);

        // voltageDrive(uL, uR);
    }
}