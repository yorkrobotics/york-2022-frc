// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.
Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.DriveWithPositionControl;
import frc.robot.commands.RunLifter;
import frc.robot.commands.ShootBall;
import frc.robot.commands.SwitchDriveMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  private Lifter mLifter;
 

  // Commands
  private DriveTeleop driveTeleop;
  private DriveWithPositionControl driveWithPositionControl;
  private SwitchDriveMode switchDriveMode;
  private ShootBall shootBall;
  private RunLifter runLifter;

  // Ramsete controllers
  private PIDController mRamseteLeftController, mRamseteRightController;

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

    //Lifter subsystem
    mLifter = new Lifter();
    runLifter = new RunLifter(mLifter);
    mLifter.setDefaultCommand(runLifter);

    // Ramsete controllers
    mRamseteLeftController = new PIDController(10, 0, 0);
    mRamseteRightController = new PIDController(10, 0, 0);

    // Populate shuffleboard
    var autoTab = Shuffleboard.getTab("Autonomous");
    autoTab.add("Left Controller", mRamseteLeftController);
    autoTab.add("Right Controller", mRamseteRightController);

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
    // new JoystickButton(mController, Button.kA.value).whenPressed(driveWithPositionControl);

    //right bumper to shift up and left bumper to shift down
    new JoystickButton(mController, Button.kRightBumper.value).whenPressed(new InstantCommand(mDrive::shiftUp, mDrive));
    new JoystickButton(mController, Button.kLeftBumper.value).whenPressed(new InstantCommand(mDrive::shiftDown, mDrive));

    new JoystickButton(mController, Button.kX.value).whenPressed(switchDriveMode);

    new JoystickButton(mController, Button.kY.value).whenPressed(mLifter::switchLifterMode, mLifter);

    new JoystickButton(mController, Button.kB.value).whenPressed(mLifter::resetEncoder, mLifter);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command to run in autonomous
    mDrive.resetEncoders();
    mDrive.getOdometry().resetPosition(new Pose2d(), new Rotation2d());

    var autoVoltageConstraint= new DifferentialDriveVoltageConstraint(mDrive.getFeedForward(), mDrive.getKinematics(), 10);

    TrajectoryConfig config = new TrajectoryConfig(0.5, 1).setKinematics(mDrive.getKinematics()).addConstraint(autoVoltageConstraint);

    Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(), 
      List.of(), 
      new Pose2d(3, 0, new Rotation2d()), 
      config
    );

    RamseteController ramseteController = new RamseteController(2.0, 0.7);
    RamseteCommand testCommand = new RamseteCommand(
      testTrajectory, 
      mDrive::getPose,
      ramseteController, 
      mDrive.getFeedForward(), 
      mDrive.getKinematics(), 
      mDrive::getWheelSpeeds, 
      mRamseteLeftController, 
      mRamseteRightController, 
      mDrive::tankDriveVolts, 
      mDrive
    );

    return testCommand;
  }
}
