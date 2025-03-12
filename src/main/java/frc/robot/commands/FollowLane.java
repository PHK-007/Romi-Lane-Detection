package frc.robot.commands;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class FollowLane extends CommandBase{
    private final Drivetrain m_drive;
    private final double m_time;
    private final double m_speed;
    private double initTime;


    /**
     * 
     * @param drive drivetrain subsystem to be used
     * @param time the time to run the lane following program in seconds
     * @param speed the speed of the robot (the units should be in m/s)
     */
    public FollowLane(Drivetrain drive, double time, double speed) {
        m_drive = drive;
        m_time = time;
        m_speed = speed;

        addRequirements(drive);
    }


    @Override
    public void initialize() {
        m_drive.arcadeDrive(0, 0);
        m_drive.resetEncoders();
        initTime = Timer.getFPGATimestamp();
        // m_drive.resetPIDControllers();
    }

    @Override
    public void execute() {
        m_drive.arcadeDrive(m_speed, initTime);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - initTime >= m_time; // or don't see any lane markings
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }
}