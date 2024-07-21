// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.DriveDistancePID.DistanceUnits;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousTest extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param blockLength The length of one block, usually 50 cm
   * @param driveSpeed The speed of forward/backward drive in m/s
   * @param turnSpeed The speed of turning in the range [-1.0, 1.0]
   * @param turnDegree The degree the robot will turn, usually 90
   */
  public AutonomousTest(Drivetrain drivetrain, double blockLength, double driveSpeed, double turnSpeed, double turnDegree) {
    /* Methods
     * new DriveDistance(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
     * new TurnDegrees(turnSpeed, turnDegree, drivetrain),
     * new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
     */
    addCommands(
      new DriveDistancePID(-5, 15, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(-turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(-turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(-turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(-turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      // new TurnDegrees(turnSpeed, 2 * turnDegree, drivetrain),
      new WaitTime(0.25, drivetrain),
      new DriveDistancePID(-driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(-turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, 2 * blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain),
      new TurnDegrees(turnSpeed, turnDegree, drivetrain),
      new DriveDistancePID(driveSpeed, blockLength, DistanceUnits.CENTIMETERS, drivetrain)
    );
  }
}