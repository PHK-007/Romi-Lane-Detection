// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveDistancePID;
import frc.robot.commands.DriveDistancePID.DistanceUnits;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.Drivetrain;


/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    
    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
    }
    
    /**
    * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
    * that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        
        
        // SmartDashboard.putNumber("Left Distance", Drivetrain.getInstance().getLeftDistanceCM());
        // SmartDashboard.putNumber("Right Distance", Drivetrain.getInstance().getRightDistanceCM());
        
        // SmartDashboard.putNumber("Current Bearing: ", Drivetrain.getInstance().getCurrentBearing());
        // SmartDashboard.putNumber("Target Bearing: ", Drivetrain.getInstance().getTargetBearing());
        
        // SmartDashboard.putNumber("Left Speed", Drivetrain.getInstance().getLeftSpeed());
        // SmartDashboard.putNumber("Right Speed", Drivetrain.getInstance().getRightSpeed());

        // SmartDashboard.putNumber("uL", Drivetrain.getInstance().getLeftVoltage());
        // SmartDashboard.putNumber("uR", Drivetrain.getInstance().getRightVoltage());
        
        
        // SmartDashboard.putString("Current Turn Direction", Drivetrain.getInstance().getCurrentTurnDirection().toString());
        // SmartDashboard.putNumber("Net Error", Drivetrain.getInstance().getNetGyroError());


        Vision.getInstance().update();
        // System.out.println("testValue from NT: " + Vision.getInstance().getMyX());
        SmartDashboard.putNumberArray("Left Lane Edges from Romi", Vision.getInstance().getLeftLaneEdges());
        SmartDashboard.putNumberArray("Right Lane Edges from Romi", Vision.getInstance().getRightLaneEdges());
    }
    
    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {}
    
    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        Drivetrain.getInstance().resetGyro();
        Drivetrain.getInstance().resetEncoders();
        
        // Get selected routine from the SmartDashboard
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }
    
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // schedule the autonomous command (example)
        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.schedule();
        // }
    }
    
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running which will
        // use the default command which is ArcadeDrive. If you want the autonomous
        // to continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        CommandScheduler.getInstance().schedule();
    }
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        
    }
    
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
    
    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
