// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToBearing extends CommandBase {
    private final Drivetrain m_drive;
    private final double m_bearing;
    private double m_speed;
    
    
    /**
    * Creates a new Turnbearing. This command will turn your robot for a desired rotation (in
    * bearing) and rotational speed.
    *
    * @param speed The speed which the robot will drive. Negative is in reverse.
    * @param bearing bearing for the robot to turn to. Must be between 0-359
    * @param drive The drive subsystem on which this command will run
    */
    public TurnToBearing(double speed, double bearing, Drivetrain drive) {
        m_bearing = bearing;
        m_speed = speed;
        m_drive = drive;
        addRequirements(drive);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Set motors to stop, read encoder values for starting point
        m_drive.arcadeDrive(0, 0);
        m_drive.resetEncoders();
        m_drive.setTargetBearing(m_bearing);
        
        m_speed = Math.abs(Drivetrain.getInstance().getTargetBearing() - Drivetrain.getInstance().getCurrentBearing()) <= 180? -m_speed : m_speed;
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // m_speed = Math.abs(Drivetrain.getInstance().getTargetBearing() - Drivetrain.getInstance().getCurrentBearing()) < 180? -m_speed : m_speed;
        m_drive.arcadeDrive(0, m_speed);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {    
        // Compare current bearing and target bearing
        return Math.abs(Drivetrain.getInstance().getCurrentBearing() - Drivetrain.getInstance().getTargetBearing()) <= 5;
    }
}
