// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.RobotTour;
import frc.robot.commands.FollowLane;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final Drivetrain m_drivetrain = new Drivetrain();
    private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
    
    // Assumes a gamepad plugged into channnel 0
    private final Joystick m_controller = new Joystick(1);

    private final CommandXboxController driver = new CommandXboxController(0);
    
    // Create SmartDashboard chooser for autonomous routines
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
    // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
    // By default, the following are available (listed in order from inside of the board to outside):
    // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
    // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
    // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
    // - PWM 2 (mapped to Arduino Pin 21)
    // - PWM 3 (mapped to Arduino Pin 22)
    //
    // Your subsystem configuration should take the overlays into account
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        //----------------------------------------------------- A U T O N -----------------------------------------------------
        
        // Setup SmartDashboard options
        
        m_chooser.setDefaultOption("Follow Lane", new FollowLane(Drivetrain.getInstance(), 50, 0.175));
        m_chooser.addOption("Robot Tour", new RobotTour(Drivetrain.getInstance(), 49, 0.4, 0.3, 90));
        SmartDashboard.putData(m_chooser);
    }
    
    
    /**
    * Use this method to define your button->command mappings. Buttons can be created by
    * instantiating a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} 
    * or {@link XboxController}), and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {
        // Default command is arcade drive. This will run unless another command
        // is scheduled over it.
        Drivetrain.getInstance().setDefaultCommand(getArcadeDriveCommand());
        
        // Example of how to use the onboard IO
        Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
        onboardButtonA
            .onTrue(new PrintCommand("Button A Pressed"))
            .onFalse(new PrintCommand("Button A Released"));
        
        driver.a().onTrue(Drivetrain.getInstance().setVelocityCommand(0).alongWith(Commands.print("Pressing A")));
        driver.b().onTrue(Drivetrain.getInstance().setVelocityCommand(5).alongWith(Commands.print("Pressing B")));
        driver.x().onTrue(Drivetrain.getInstance().setVelocityCommand(10).alongWith(Commands.print("Pressing X")));
        driver.y().onTrue(Drivetrain.getInstance().setVelocityCommand(15).alongWith(Commands.print("Pressing Y")));

        driver.rightBumper().or(driver.leftBumper()).onTrue(Drivetrain.getInstance().setDriveVoltage(0));
    }
            
            
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
            

    /**
    * Use this to pass the teleop command to the main {@link Robot} class.
    *
    * @return the command to run in teleop
    */
    public Command getArcadeDriveCommand() {
        return new ArcadeDrive(Drivetrain.getInstance(), () -> -driver.getRawAxis(1), () -> -driver.getRawAxis(4) / 2);
    }
}     