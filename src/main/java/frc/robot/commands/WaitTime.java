// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitTime extends CommandBase {
    private final double m_duration;
    private final Drivetrain m_drive;
    private long m_startTime;
    
    /**
    * Creates a new WaitTime. This command will stop the robot for the specified amount of time
    *
    * @param time How much time to wait in seconds
    * @param drive The drivetrain subsystem on which this command will run
    */
    public WaitTime(double time, Drivetrain drive) {
        m_duration = time * 1000;
        m_drive = drive;
        addRequirements(drive);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_startTime = System.currentTimeMillis();
        m_drive.arcadeDrive(0, 0);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.arcadeDrive(0, 0);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - m_startTime) >= m_duration;
    }
}
