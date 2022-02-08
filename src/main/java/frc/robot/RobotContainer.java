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
import frc.robot.commands.RunLifter;
import frc.robot.commands.ShootBall;
import frc.robot.commands.SwitchDriveMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GearShift;
import frc.robot.subsystems.Lifter;
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
  // Subsystems
  private DriveTrain mDrive;
  private GearShift mGearShift;
  private Shooter mShooter;
  private Lifter mLifter;

  // Commands
  private DriveTeleop driveTeleop;
  private DriveWithPositionControl driveWithPositionControl;
  private SwitchDriveMode switchDriveMode;
  private GearShiftDown gearShiftDown;
  private GearShiftUp gearShiftUp;
  private ShootBall shootBall;
  private RunLifter runLifter;

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

    // GearShift subsystem
    mGearShift = new GearShift();
    gearShiftDown = new GearShiftDown(mGearShift, mDrive);
    gearShiftUp = new GearShiftUp(mGearShift, mDrive);

    // Shooter subsystem
    mShooter = new Shooter();
    shootBall = new ShootBall(mShooter);
    // mShooter.setDefaultCommand(shootBall);

    //Lift subsystem
    mLifter = new Lifter();
    runLifter = new RunLifter(mLifter);
    mLifter.setDefaultCommand(runLifter);

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
    JoystickButton button_B = new JoystickButton(mController, Button.kB.value);
    button_B.whenPressed(gearShiftDown);
    JoystickButton button_Y = new JoystickButton(mController, Button.kY.value);
    button_Y.whenPressed(gearShiftUp);
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
