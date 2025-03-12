// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Motor constants
    public static double motor1P;
    public static double motor1I;
    public static double motor1D;

    public static double motor2P = 0.01; // Need to test at other speeds
    public static double motor2I = 0.04; // Need to test at other speeds
    public static double motor2D;

    public static double motor3P;
    public static double motor3I;
    public static double motor3D;

    public static double motor4P;
    public static double motor4I;
    public static double motor4D;
}