// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegrees extends CommandBase {
    private final Drivetrain m_drive;
    private final double m_degrees;
    private final double m_speed;
    final double targetTurningDistance;
    
    double inchPerDegree = Math.PI * 5.551 / 360;
    double cmPerDegree = Math.PI * 14.1 / 360;
    
    /**
    * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
    * degrees) and rotational speed.
    *
    * @param speed The speed which the robot will drive. Positive is clockwise
    * @param degrees Degrees to turn. Leverages encoders to compare distance.
    * @param drive The drive subsystem on which this command will run
    */
    public TurnDegrees(double speed, double degrees, Drivetrain drive) {
        m_degrees = Math.abs(degrees);
        m_speed = (degrees < 0) ? -speed : speed;
        m_drive = drive;
        targetTurningDistance = m_degrees * cmPerDegree;
        addRequirements(drive);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Set motors to stop, read encoder values for starting point
        m_drive.arcadeDrive(0, 0);
        m_drive.resetEncoders();
        SmartDashboard.putNumber("Target Rotation", targetTurningDistance);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.arcadeDrive(0, -m_speed);
        
        SmartDashboard.putNumber("Avg Turning Distance", getAverageTurningDistance());
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
        
        SmartDashboard.putNumber("Percent Error", Math.abs(getAverageTurningDistance() - targetTurningDistance) / targetTurningDistance * 100);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        /* Need to convert distance travelled to degrees. The Standard
        Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
        has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
        or 5.551 inches. We then take into consideration the width of the tires.
        */
        
        // Compare distance travelled from start to distance based on degree turn
        return getAverageTurningDistance() >= targetTurningDistance;
    }
    
    private double getAverageTurningDistance() {
        double leftDistance = Math.abs(m_drive.getLeftDistanceCM());
        double rightDistance = Math.abs(m_drive.getRightDistanceCM());
        return (leftDistance + rightDistance) / 2.0;
    }
}
