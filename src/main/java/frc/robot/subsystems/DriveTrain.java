// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Ultrasonic;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;




import edu.wpi.first.wpilibj2.*;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  

  private WPI_TalonSRX left = new WPI_TalonSRX(RobotMap.leftDrivePort);
  private WPI_TalonSRX right = new WPI_TalonSRX(RobotMap.rightDrivePort);
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private Ultrasonic ultrasonic = new Ultrasonic(RobotMap.ultrasonic1, RobotMap.ultrasonic2);
  private static DriveTrain instance;
  //private double TIKS_TO_INCHES;
  private static double kTrackwidth;
  private int kTiksPerRotation;
  private double kTiksToInches;
  private double kWheelCircumMeters = 2 * Math.PI * Units.inchesToMeters(RobotMap.kWheelRadiusInches);


  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(kTrackwidth));
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(RobotMap.kS, RobotMap.kV, RobotMap.kA);

  private PIDController leftPID = new PIDController(RobotMap.kP, RobotMap.kI, RobotMap.kD);
  private PIDController rightPID = new PIDController(RobotMap.kP, RobotMap.kI, RobotMap.kD);

  Pose2d pose = new Pose2d();
  
  public DriveTrain() {
    left.setInverted(true);

    right.configFactoryDefault();
    left.configFactoryDefault();
    right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    //ultrasonic.setAutomaticMode(true);

    gyro.zeroYaw();
  }

  public static DriveTrain getInstance(){
    if(instance == null){
      instance = new DriveTrain();
    }
    return instance;
  }

  public void tankDrive(double lPower, double rPower){
    left.set(ControlMode.PercentOutput, lPower);
    right.set(ControlMode.PercentOutput, rPower);
  }

  public void resetEncoders(){
    left.setSelectedSensorPosition(0,0,10); 
    right.setSelectedSensorPosition(0,0,10); 
  }

  /*public double getDistance(){
    return ((left.getSelectedSensorPosition(0) + right.getSelectedSensorPosition(0))/2) * TIKS_TO_INCHES;
  }*/

  /*public double getUltrasonicDistance(){
    return ultrasonic.getRangeInches();
  }*/
  
  public void resetGyro(){
    gyro.zeroYaw();
  }

  public double getAngle(){
    return gyro.getYaw();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public void setOutput(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts); 
    System.out.println("distance right: " + getRightDistance());
    System.out.println("distance left: " + getLeftDistance());
    System.out.println("position: " + returnPose());
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        left.getSelectedSensorVelocity() / RobotMap.kGearRatio * 2 * Math.PI * Units.inchesToMeters(RobotMap.kWheelRadiusInches) / 60,
       right.getSelectedSensorVelocity() / RobotMap.kGearRatio * 2 * Math.PI * Units.inchesToMeters(RobotMap.kWheelRadiusInches) / 60
    );
  }

  public double getRightDistance() { //encoder positions are inverted for blue robot
    return (
      right.getSelectedSensorPosition() * kWheelCircumMeters / (4096)
      );
  }
  public double getLeftDistance() {
    return (
      left.getSelectedSensorPosition() * kWheelCircumMeters / (4096)
      );
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedForward;
  }

  public PIDController getLeftPID() {
    return leftPID;
  }

  public PIDController getRightPID() {
    return rightPID;
  }

  public void resetOdometry(Pose2d pose) {
    left.setSelectedSensorPosition(0, 0, 10);
    right.setSelectedSensorPosition(0, 0, 10); 

    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void zeroHeading() {
    gyro.reset();
  }

  
  public SimpleMotorFeedforward getFeedForward() {
    return feedForward;
  }
  public PIDController returnLeftPID() {
    return leftPID;
  }
  public PIDController returnRightPID() {
    return rightPID;
  }

  public Pose2d returnPose() {
    return odometry.getPoseMeters();
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), getRightDistance(), getLeftDistance());
    tankDrive(RobotContainer.getJoy().getY(), RobotContainer.getJoy1().getY());
  }
}