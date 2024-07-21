// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Timer;

// // Use this class to use PID to control the bearing
// public class PIDController {

//     // PID Constants
//     private double kP = 0.0085;
//     private double kI = 0;
//     private double kD = 0;

//     private double e;
//     private double eIntegral;
//     private double u;
//     private double deltaT;
//     private double prevT;
//     private double currentT;

//     public PIDController(double kP, double kI, double kD) {
//         this.kP = kP;
//         this.kI = kI;
//         this.kD = kD;
//         e = eIntegral = u = deltaT = 0;
//         currentT = Timer.getFPGATimestamp();
//         prevT = Timer.getFPGATimestamp();
//     }


//     public double getTargetVoltage(double targetSpeed, double currentSpeed) {
//         currentT = Timer.getFPGATimestamp();
//         e = targetSpeed - currentSpeed;
//         deltaT = currentT - prevT;
//         eIntegral += e * deltaT;
//         u = (this.kP * e) + (this.kI * eIntegral);
//         prevT = currentT;
//         return u;
//     }


//     public void reset() {
//         currentT = Timer.getFPGATimestamp();
//         prevT = Timer.getFPGATimestamp();
//         eIntegral = 0;
//         deltaT = 0;
//     }

//     public double getError() {
//         return e;
//     }

//     public double getErrorIntegral() {
//         return eIntegral;
//     }
// }