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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.CommandBuilder;
import frc.robot.autonomous.TrajectoryBuilder;
import frc.robot.commands.CorrectOdometry;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.RotateToTarget;
import frc.robot.commands.ShootTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PyCamera;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private DriveTrain mDrive;
  private Climb mClimb;
  private Shooter mShooter;
  private Intake mIntake;
  private Tower mTower;
  private ColorSensor mColorSensor;
  private Conveyor mConveyor;

  // Commands
  private DriveTeleop driveTeleop;

  // XboxController
  public static XboxController mController;

  // Camera
  private PyCamera pycam;

  // AutoRoutine
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

    // Lifter subsystem
    mClimb = Climb.getInstance();
    mClimb.setDefaultCommand(new RunCommand(()-> mClimb.runClimbWithJoystick(mController), mClimb));
    
    // Intake
    mIntake = Intake.getInstance();

    // Shooter
    mShooter = Shooter.getInstance();

    // Conveyor
    mConveyor = Conveyor.getInstance();

    // Tower
    mTower = Tower.getInstance();
    mTower.setDefaultCommand(new RunCommand(()-> mTower.runTowerWithController(mController), mTower));

    //pycam
    pycam = new PyCamera();

    // Color sensor
    mColorSensor = ColorSensor.getInstance();

    // Autonomous
    mTrajectoryBuilder = new TrajectoryBuilder(Constants.PATH_FOLDER);
    mCommandBuilder = new CommandBuilder(mIntake, mShooter, mConveyor);
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
    /**
     * Buttons mapping
     * A:
     * B: 
     * X:
     * Y:
     * LeftBumper: Shift up
     * RightBumper: Shift down
     * LeftStick: Switch drive mode
     * RightStick: 
     */

    new JoystickButton(mController, Button.kRightBumper.value).whenPressed(mDrive::shiftToHighGear, mDrive);
    new JoystickButton(mController, Button.kLeftBumper.value).whenPressed(mDrive::shiftToLowGear, mDrive);

    new JoystickButton(mController, Button.kLeftStick.value).whenPressed(mDrive::switchDriveMode, mDrive);

    new JoystickButton(mController, Button.kRightStick.value).whenPressed(mClimb::switchClimbMode, mClimb);

    // new JoystickButton(mController, Button.kB.value).whileHeld(new RotateToTarget(mDrive, pycam));

    new JoystickButton(mController, Button.kX.value).whenPressed(() -> {
      mIntake.runRoller(Constants.INTAKE_ROLLER_SPEED);
      mConveyor.runConveyor(Constants.CONVEYOR_SPEED);
    }, mIntake, mConveyor).whenReleased(() -> {
      mIntake.stopRoller();
      mConveyor.stopConveyor();
    }, mIntake, mConveyor);
    new JoystickButton(mController, Button.kY.value).whenPressed(() -> {
      mIntake.runRoller(-Constants.INTAKE_ROLLER_SPEED);
      mConveyor.runConveyor(-Constants.CONVEYOR_SPEED);
    }, mIntake, mConveyor).whenReleased(() -> {
      mIntake.stopRoller();
      mConveyor.stopConveyor();
    }, mIntake, mConveyor);

    
    new JoystickButton(mController, Button.kA.value).whenPressed(new InstantCommand(()->mClimb.goHome(), mClimb));
    new JoystickButton(mController, Button.kB.value).whenPressed(new InstantCommand(()->mTower.goHome(), mTower));

    new POVButton(mController, 90).whenPressed(mIntake::deploy, mIntake);
    new POVButton(mController, 270).whenPressed(mIntake::retract, mIntake);

    new POVButton(mController, 180).whenPressed(() -> {
      if (mShooter.isShooting()) mShooter.stopShooter();
      else mShooter.runShooter(SmartDashboard.getNumber("Test speed", 0.0));
    });

    // new POVButton(mController, 180).whenPressed(new ShootTarget(pycam, mShooter, mTower));


    new POVButton(mController, 0).whenPressed(mTower::switchActuatorMode, mTower);

    // new JoystickButton(mController, Button.kY.value).whenPressed(mDrive::turnToTarget, mDrive);
    // new JoystickButton(mController, Button.kY.value).whenPressed(new CorrectOdometry(pycam, mDrive));

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
