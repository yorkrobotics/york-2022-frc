// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DriveForwardDistance;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.DriveWithVelocityControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VelocityController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // DriveTrain Declare
  private DriveTrain m_drive;
  private DriveTeleop driveTeleop;
  private DriveForwardTimed driveForwardTimed;

  private DriveForwardDistance dfd;

  private VelocityController m_velocityController;
  private DriveWithVelocityControl driveWithVelocityControl;

  public static XboxController m_controller;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // m_drive = new DriveTrain();
    
    // driveTeleop = new DriveTeleop(m_drive);
    // driveTeleop.addRequirements(m_drive);

    // m_drive.setDefaultCommand(driveTeleop);

    // driveForwardTimed = new DriveForwardTimed(m_drive);
    // driveForwardTimed.addRequirements(m_drive);

    m_controller = new XboxController(Constants.CONTROLLER_NUMBER);

    // dfd = new DriveForwardDistance(m_drive);
    // dfd.addRequirements(m_drive);

    // Testing velocity controller
    m_velocityController = new VelocityController();
    driveWithVelocityControl = new DriveWithVelocityControl(m_velocityController);
    driveWithVelocityControl.addRequirements(m_velocityController);
    m_velocityController.setDefaultCommand(driveWithVelocityControl);


    // Configure the button bindings
    configureButtonBindings(
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {  
    // JoystickButton button_A = new JoystickButton(m_controller, Button.kA.value);
    // button_A.whenPressed(new DriveForwardDistance(m_drive));   

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command to run in autonomous
    return dfd;
  }
}
