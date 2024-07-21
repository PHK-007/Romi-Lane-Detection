// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.DriveDistance.DistanceUnits;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotTour extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public RobotTour(Drivetrain drivetrain, double blockLength, double driveSpeed, double turnSpeed, double turnDegree) {
    /* BASIC METHODS
     * new DriveDistance(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
     * new TurnDegrees(turnSpeed, turnDegree, drivetrain),
     * new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
     * 
     * new RightUTurn(driveSpeed, turnSpeed, turnDegree, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
     * new LeftUTurn(driveSpeed, turnSpeed, turnDegree, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
     */
    addCommands(
      new DriveDistance(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(turnSpeed, turnDegree, drivetrain),
      new DriveDistance(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain)

    );
  }
}