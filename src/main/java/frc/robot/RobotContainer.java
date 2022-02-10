// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.DriveWithPositionControl;
import frc.robot.commands.ShootBall;
import frc.robot.commands.SwitchDriveMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private DriveTrain mDrive;
  private Shooter mShooter;

  // Commands
  private DriveTeleop driveTeleop;
  private DriveWithPositionControl driveWithPositionControl;
  private SwitchDriveMode switchDriveMode;
  private ShootBall shootBall;

  // XboxController
  public static XboxController mController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Driver's controller
    mController = new XboxController(Constants.CONTROLLER_NUMBER);

    // DriveTrain subsystem
    mDrive = new DriveTrain();
    driveTeleop = new DriveTeleop(mDrive);
    mDrive.setDefaultCommand(driveTeleop);

    driveWithPositionControl = new DriveWithPositionControl(mDrive);
    switchDriveMode = new SwitchDriveMode(mDrive);

    // Shooter subsystem
    mShooter = new Shooter();
    shootBall = new ShootBall(mShooter);
    // mShooter.setDefaultCommand(shootBall);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {  
    JoystickButton button_A = new JoystickButton(mController, Button.kA.value);
    button_A.whenPressed(driveWithPositionControl);

    JoystickButton right_bumper = new JoystickButton(mController, Button.kRightBumper.value);
    right_bumper.whenPressed(new InstantCommand(mDrive::shiftUp, mDrive));
    JoystickButton left_bumper = new JoystickButton(mController, Button.kLeftBumper.value);
    left_bumper.whenPressed(new InstantCommand(mDrive::shiftDown, mDrive));

    JoystickButton button_X = new JoystickButton(mController, Button.kX.value);
    button_X.whenPressed(switchDriveMode);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command to run in autonomous
    return driveWithPositionControl;
  }
}
