// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.RunLifter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lifter;
import frc.robot.commands.ShootTarget;
import frc.robot.subsystems.FeedIntake;
import frc.robot.subsystems.PyCamera;
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
  private Lifter mLifter;
  private Shooter m_shooter;
  private FeedIntake m_feed;

  // Commands
  private DriveTeleop driveTeleop;
  private RunLifter runLifter;

  // Ramsete controllers
  private PIDController mRamseteLeftController, mRamseteRightController;

  // XboxController
  public static XboxController mController;


  public static XboxController m_controller;
  //camera
  private PyCamera pycam;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Driver's controller
    mController = new XboxController(Constants.CONTROLLER_NUMBER);

    // DriveTrain subsystem
    mDrive = new DriveTrain();
    driveTeleop = new DriveTeleop(mDrive);
    mDrive.setDefaultCommand(driveTeleop);

    //Lifter subsystem
    mLifter = new Lifter();
    runLifter = new RunLifter(mLifter);
    mLifter.setDefaultCommand(runLifter);

    // Ramsete controllers
    mRamseteLeftController = new PIDController(5.7673, 0, 0);
    mRamseteRightController = new PIDController(5.7673, 0, 0);

    // Populate shuffleboard
    var autoTab = Shuffleboard.getTab("Autonomous");
    autoTab.add("Left Controller", mRamseteLeftController);
    autoTab.add("Right Controller", mRamseteRightController);
    
    //feed
    m_feed = new FeedIntake();

    //Shooter stuff
    m_shooter = new Shooter();

    //pycam
    pycam = new PyCamera();

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

    new JoystickButton(mController, Button.kLeftStick.value).whenPressed(mDrive::switchDriveMode, mDrive);

    new JoystickButton(mController, Button.kRightStick.value).whenPressed(mLifter::switchLifterMode, mLifter);

    new JoystickButton(mController, Button.kA.value).whenPressed(()->{mDrive.setPosition(2);}, mDrive);
    
    // B TBD
    new JoystickButton(mController, Button.kB.value).whenReleased(new ShootTarget(pycam, m_shooter));

    // X to feed
    new JoystickButton(mController, Button.kX.value).whileHeld(()->{
      m_shooter.setSpeed(0.5);
    });
    
    // Y to shoot
    new JoystickButton(mController, Button.kY.value).whileHeld(()->{
      m_feed.setSpeed(0.5);
    });

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

    var autoVoltageConstraint= new DifferentialDriveVoltageConstraint(mDrive.getFeedForward(), mDrive.getKinematics(), Constants.kMaxVoltage);

    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxVelocity, Constants.kMaxAcceleration).setKinematics(mDrive.getKinematics()).addConstraint(autoVoltageConstraint);

    Trajectory toHomePosition = TrajectoryGenerator.generateTrajectory(
      mDrive.getPose(), 
      List.of(), 
      new Pose2d(0, 0, new Rotation2d()), 
      config
    );

    Trajectory heartTrajectory = new Trajectory();
    
    String heartJSON = "paths/Heart.wpilib.json";

    try {
      Path heartPath = Filesystem.getDeployDirectory().toPath().resolve(heartJSON);
      heartTrajectory = TrajectoryUtil.fromPathweaverJson(heartPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + heartJSON, ex.getStackTrace());
      return null;
   }

    RamseteController ramseteController = new RamseteController(2.0, 0.7);
    RamseteCommand testCommand = new RamseteCommand(
      heartTrajectory, 
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
