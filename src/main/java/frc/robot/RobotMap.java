package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class RobotMap {

    //motor ports
    public static int leftDrivePort = 0;
    public static int rightDrivePort = 1;

    //joystick ports
	public static int joy = 0;
    public static int joy1 = 1;   

    //ultrasonic ports
    public static int ultrasonic1 = 0;
    public static int ultrasonic2 = 1;

    public static int navXPort = 0;
    public static  double kGearRatio;
    public static  double kWheelRadiusInches;

    public static int leftJoy = 0;
    
    public static double kS = 1.55;
    public static double kV = 4.93; //change after running frc characterization data logger
    public static double kA = 0.588;

    public static double kP = 1.65;
    public static double kI;
    public static double kD;
    
}