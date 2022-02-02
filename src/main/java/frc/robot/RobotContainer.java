// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.DriveWithPositionControl;
import frc.robot.commands.GearShiftDown;
import frc.robot.commands.GearShiftUp;
import frc.robot.commands.ShootBall;
import frc.robot.commands.feed;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeedIntake;
import frc.robot.subsystems.GearShift;
import frc.robot.subsystems.Shooter;
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
  private DriveWithPositionControl driveWithPositionControl;
  private GearShiftDown gearShiftDown;
  private GearShiftUp gearShiftUp;
  private GearShift gearShift;

  //Shooter
  private Shooter m_shooter;
  private ShootBall shootBall;

  //Feed
  private FeedIntake m_feed;
  private feed feed;

  public static XboxController m_controller;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //drive's controller
    m_controller = new XboxController(Constants.CONTROLLER_NUMBER);

    //Driving during teleop.
    m_drive = new DriveTrain();
    driveTeleop = new DriveTeleop(m_drive);
    driveTeleop.addRequirements(m_drive);
    m_drive.setDefaultCommand(driveTeleop);


    driveWithPositionControl = new DriveWithPositionControl(m_drive);
    driveWithPositionControl.addRequirements(m_drive);

    gearShift = new GearShift();
    gearShiftDown = new GearShiftDown(gearShift);
    gearShiftUp = new GearShiftUp(gearShift);


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
    JoystickButton button_A = new JoystickButton(m_controller, Button.kA.value);
    button_A.whenPressed(new DriveWithPositionControl(m_drive));
    JoystickButton button_B = new JoystickButton(m_controller, Button.kB.value);
    button_B.whenPressed(new GearShiftUp(gearShift));
    JoystickButton button_Y = new JoystickButton(m_controller, Button.kY.value);
    button_Y.whenPressed(new GearShiftDown(gearShift));
    JoystickButton button_X = new JoystickButton(m_controller, Button.kX.value);
    button_X.whenHeld(new feed(m_feed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command to run in autonomous
    return shootBall;
  }
}
