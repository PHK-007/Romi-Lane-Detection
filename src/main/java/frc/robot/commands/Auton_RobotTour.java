// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.AutonTemplates.RightUTurn;
import frc.robot.commands.AutonTemplates.LeftUTurn;
import frc.robot.commands.DriveDistancePID.DistanceUnits;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auton_RobotTour extends SequentialCommandGroup {

  /**
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public Auton_RobotTour(Drivetrain drivetrain, double blockLength, double driveSpeed, double turnSpeed, double turnDegree) {
    /* BASIC METHODS
     * new DriveDistance(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
     * new TurnDegrees(turnSpeed, turnDegree, drivetrain),
     * new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
     * 
     * new RightUTurn(driveSpeed, turnSpeed, turnDegree, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
     * new LeftUTurn(driveSpeed, turnSpeed, turnDegree, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
     */
    addCommands(
      new DriveDistancePID(driveSpeed, blockLength / 2 + 10, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(turnSpeed, turnDegree, drivetrain),
      new LeftUTurn(driveSpeed, turnSpeed, turnDegree, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(-turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new WaitTime(0.5, drivetrain),
      new DriveDistancePID(-driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(-turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, 2 * blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(-turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new WaitTime(0.5, drivetrain),
      new DriveDistancePID(-driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(-turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength - 10, DistanceUnits.CENTIMETERS, drivetrain)
    );
  }
}