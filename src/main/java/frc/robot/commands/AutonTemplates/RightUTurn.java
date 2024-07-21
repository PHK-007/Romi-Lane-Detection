package frc.robot.commands.AutonTemplates;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistancePID;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.DriveDistancePID.DistanceUnits;
import frc.robot.subsystems.Drivetrain;

public class RightUTurn extends SequentialCommandGroup {
    public RightUTurn (double driveSpeed, double turnSpeed, double turnDegree, double blockLength, DistanceUnits unit, Drivetrain drive) {
        addCommands(
            new DriveDistancePID(driveSpeed, blockLength, unit, drive),
            new TurnDegrees(turnSpeed, turnDegree, drive),
            new DriveDistancePID(driveSpeed, blockLength, unit, drive),
            new TurnDegrees(turnSpeed, turnDegree, drive),
            new DriveDistancePID(driveSpeed, blockLength, unit, drive)
        );
    }
}