// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.CommandBuilder;
import frc.robot.autonomous.TrajectoryBuilder;
import frc.robot.commands.CorrectOdometry;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.RotateToTarget;
import frc.robot.commands.RunLifter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PyCamera;
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
  private Lifter mLifter;
  private Shooter m_shooter;
  private Intake mIntake;

  // Commands
  private DriveTeleop driveTeleop;
  private RunLifter runLifter;

  // XboxController
  public static XboxController mController;

  //camera
  private PyCamera pycam;

  //Test AutoRoutine
  private TrajectoryBuilder mTrajectoryBuilder;
  private CommandBuilder mCommandBuilder;
  private SendableChooser<AutoRoutine> mAutoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Driver's controller
    mController = new XboxController(Constants.CONTROLLER_PORT);

    // DriveTrain subsystem
    mDrive = DriveTrain.getInstance();
    driveTeleop = new DriveTeleop(mDrive);
    mDrive.setDefaultCommand(driveTeleop);

    //Lifter subsystem
    // mLifter = Lifter.getInstance();
    // runLifter = new RunLifter(mLifter);
    // mLifter.setDefaultCommand(runLifter);
    
    //feed
    // mIntake = Intake.getInstance();

    //Shooter stuff
    // m_shooter = Shooter.getInstance();

    //pycam
    pycam = new PyCamera();

    // Autonomous
    mTrajectoryBuilder = new TrajectoryBuilder(Constants.PATH_FOLDER);
    mCommandBuilder = new CommandBuilder();
    mAutoChooser = new SendableChooser<AutoRoutine>();

    // Populate shuffleboard
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    autoTab.add(mAutoChooser);

    mCommandBuilder.displayRoutines(mAutoChooser);


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
    new JoystickButton(mController, Button.kRightBumper.value).whenPressed(mDrive::shiftToHighGear, mDrive);
    new JoystickButton(mController, Button.kLeftBumper.value).whenPressed(mDrive::shiftToLowGear, mDrive);

    new JoystickButton(mController, Button.kLeftStick.value).whenPressed(mDrive::switchDriveMode, mDrive);

    // new JoystickButton(mController, Button.kRightStick.value).whenPressed(mLifter::switchLifterMode, mLifter);

    new JoystickButton(mController, Button.kA.value).whenPressed(()->{
      mDrive.setRotation(30);
    });
    
    // B TBD
    // new JoystickButton(mController, Button.kB.value).whenReleased(new ShootTarget(pycam, m_shooter));
    new JoystickButton(mController, Button.kB.value).whileHeld(new RotateToTarget(mDrive, pycam));

    // X to feed
    new JoystickButton(mController, Button.kX.value).whileHeld(()->{
      m_shooter.runShooter(0.5);
    });
    
    // Y to shoot
    new JoystickButton(mController, Button.kY.value).whenPressed(mDrive::turnToTarget, mDrive);
    // new JoystickButton(mController, Button.kY.value).whenPressed(new CorrectOdometry(pycam, mDrive));
    // new JoystickButton(mController, Button.kY.value).whileHeld(()->{
    //   m_feed.setSpeed(0.5);
    // });
    }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command to run in autonomous
    return mAutoChooser.getSelected().getCommand(mTrajectoryBuilder, mDrive::makeTrajectoryToCommand);
  }
}
