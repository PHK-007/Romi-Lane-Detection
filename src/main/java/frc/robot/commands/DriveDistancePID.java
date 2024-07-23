// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistancePID extends CommandBase {
    private final Drivetrain m_drive;
    private final double m_distance;
    private final double m_speed;
    private final DistanceUnits m_unit;
    private double initTime;
    
    public enum DistanceUnits {CENTIMETERS, INCHES}
    
    
    /**
    * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
    * a desired speed.
    *
    * @param speed The speed at which the robot will drive in m/s
    * @param distance The distance the robot will drive
    * @param unit The unit of the distance the robot will drive (cm or inches)
    * @param drive The drivetrain subsystem on which this command will run
    */
    public DriveDistancePID(double speed, double distance, DistanceUnits unit, Drivetrain drive) {
        // convert the distance parameter to cm and make it positive
        m_distance = (unit == DistanceUnits.CENTIMETERS) ? Math.abs(distance) : Math.abs(distance * 2.54);
        m_speed = (distance < 0) ? -speed : speed; // negate speed if distance is negative
        m_drive = drive;
        m_unit = unit;
        
        addRequirements(drive);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.arcadeDrive(0, 0);
        m_drive.resetEncoders();
        initTime = Timer.getFPGATimestamp();
        m_drive.resetPIDControllers();

        // m_drive.setVelocity(m_speed);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // m_drive.arcadeDrive(m_speed, 0);
        
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
        
        SmartDashboard.putNumber("Run Time", Timer.getFPGATimestamp() - initTime);
        
        // if (m_unit == DistanceUnits.CENTIMETERS) {
        //     SmartDashboard.putString("Distance", m_drive.getAverageDistanceCM() + "cm");
        // } else if (m_unit == DistanceUnits.INCHES) {
        //     SmartDashboard.putString("Distance", m_drive.getAverageDistanceInch() + "inches");
        // }
    }
            
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Compare distance travelled from start to desired distance
        return Math.abs(m_drive.getAverageDistanceCM()) >= m_distance;
    }
}