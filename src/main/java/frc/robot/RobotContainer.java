// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.*;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;

/** Add your docs here. */
public class RobotContainer {
    private static Joystick joy = new Joystick(RobotMap.joy);
    private static Joystick joy1 = new Joystick(RobotMap.joy1);
    private static DriveTrain m_drive = new DriveTrain();

    public static Joystick getJoy(){
        return joy;
    }

    public static Joystick getJoy1(){
        return joy1;
    }

    public static DriveTrain getDrive(){
        return m_drive;
    }

  //private Command m_autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  //joystick and subsystem getters

  public static DriveTrain returnDrive() {
    return m_drive;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //add the stuff from the video here (trajectory)

      RamseteCommand command = new RamseteCommand(
        trajectory, 
        m_drive::returnPose,
        new RamseteController(2, 0.7), 
        m_drive.getKinematics(), 
        m_drive::getWheelSpeeds, 
        m_drive.returnRightPID(), 
        m_drive.returnLeftPID(), 
        m_drive::setOutput, 
        m_drive);



      m_drive.resetOdometry(trajectory.getInitialPose());

      return command.andThen(() -> m_drive.setOutput(0, 0));
  }
}