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
import frc.robot.commands.AngleTowerSetpoint;
import frc.robot.commands.AutoVisionShoot;
import frc.robot.commands.Bop;
import frc.robot.commands.Burp;
import frc.robot.commands.ControllerRunClimb;
import frc.robot.commands.HomeClimb;
import frc.robot.commands.HomeStationaryClimb;
import frc.robot.commands.RunIntakeAndConveyor;
import frc.robot.commands.ControllerRunStationaryClimb;
import frc.robot.commands.ControllerRunTower;
import frc.robot.commands.ShootBallAndAngleTowerSequence;
import frc.robot.commands.ShootBallSequence;
import frc.robot.commands.StopIntakeAndConveyor;
import frc.robot.commands.HomeTower;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.HomeTowerAndRetractIntake;
import frc.robot.commands.ReverseIntakeAndConveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PyCamera;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StationaryClimb;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private StationaryClimb mStationaryClimb;

  // Commands
  private DriveTeleop driveTeleop;

  // XboxController
  public static XboxController mainController;
  public static XboxController secondaryController;

  // Camera
  private PyCamera pycam;

  // AutoRoutine
  private TrajectoryBuilder mTrajectoryBuilder;
  private CommandBuilder mCommandBuilder;
  private SendableChooser<AutoRoutine> mAutoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Driver's controller
    mainController = new XboxController(Constants.CONTROLLER_PORT);
    
    secondaryController = new XboxController(Constants.CONTROLLER_PORT_SECONDARY);

    // DriveTrain subsystem
    mDrive = DriveTrain.getInstance();
    mDrive.setDefaultCommand(new DriveTeleop(mainController));

    // Climb
    mClimb = Climb.getInstance();
    mClimb.setDefaultCommand(new ControllerRunClimb(secondaryController));
    
    // Stationary Climb
    mStationaryClimb = StationaryClimb.getInstance();

    // Intake
    mIntake = Intake.getInstance();

    // Shooter
    mShooter = Shooter.getInstance();

    // Conveyor
    mConveyor = Conveyor.getInstance();

    // Tower
    mTower = Tower.getInstance();
    mTower.setDefaultCommand(new ControllerRunTower(mainController, secondaryController));

    //pycam
    pycam = new PyCamera();

    // Color sensor
    mColorSensor = ColorSensor.getInstance();

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
    configureSecondaryController();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {  

    /**
     * main controller
     */
    new JoystickButton(mainController, Button.kRightBumper.value).whileHeld(mStationaryClimb::ClimbDownWithBumper, mStationaryClimb);
    new JoystickButton(mainController, Button.kLeftBumper.value).whileHeld(mStationaryClimb::ClimbUpWithBumper, mStationaryClimb);

    new JoystickButton(mainController, Button.kLeftStick.value).whenPressed(mDrive::switchInvertedDriving, mDrive);

    new JoystickButton(mainController, Button.kA.value).whenPressed(new RunIntakeAndConveyor())
      .whenReleased(new StopIntakeAndConveyor());
    new JoystickButton(mainController, Button.kY.value).whenPressed(new ReverseIntakeAndConveyor())
      .whenReleased(new StopIntakeAndConveyor());
    
    new JoystickButton(mainController, Button.kX.value).whenPressed(mDrive::shiftToLowGear, mDrive);
    new JoystickButton(mainController, Button.kB.value).whenPressed(mDrive::shiftToHighGear, mDrive);

    new JoystickButton(mainController, Button.kStart.value).whenPressed(
      new ParallelCommandGroup(
        new HomeTower(),
        new InstantCommand(mShooter::stopShooter, mShooter)
      )
      );
    new JoystickButton(mainController, Button.kBack.value).whenPressed(new HomeStationaryClimb());

    new POVButton(mainController, 90).whenPressed(new DeployIntake());
    new POVButton(mainController, 270).whenPressed(new HomeTowerAndRetractIntake());

    ShootBallAndAngleTowerSequence shootBallAndAngleTowerSequence = new ShootBallAndAngleTowerSequence(0);
    SmartDashboard.putNumber("towe angle", 0);
    SmartDashboard.putNumber("velocity", 0);
    new POVButton(mainController, 180).toggleWhenPressed(
      // new InstantCommand(()->{
      //   shootBallAndAngleTowerSequence.setAngle(SmartDashboard.getNumber("towe angle", 0));
      //   shootBallAndAngleTowerSequence.setVelocity(SmartDashboard.getNumber("velocity", 0));
      // }).andThen(shootBallAndAngleTowerSequence)
      new HomeTower().andThen(new AutoVisionShoot())
    );
    
    new POVButton(mainController, 0).whenPressed(
      new ConditionalCommand(
        new SequentialCommandGroup(        
          new InstantCommand(mShooter::stopShooter, mShooter),
          new HomeTower()
        ),
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new AngleTowerSetpoint(60),
            new ShootBallSequence(50)
          )  
        ),
        mShooter::isShooting
      )
    );
  }

  private void configureSecondaryController(){
    new JoystickButton(secondaryController, Button.kB.value).whenPressed(new HomeClimb());
    new JoystickButton(secondaryController, Button.kX.value).whenPressed(new HomeTower());
    
    new JoystickButton(secondaryController, Button.kY.value).whenPressed(new ReverseIntakeAndConveyor())
      .whenReleased(new StopIntakeAndConveyor());
    new JoystickButton(secondaryController, Button.kA.value).whenPressed(new RunIntakeAndConveyor())
      .whenReleased(new StopIntakeAndConveyor());

    new JoystickButton(secondaryController, Button.kStart.value).whenPressed(new Bop())
      .whenReleased(new StopIntakeAndConveyor());
    new JoystickButton(secondaryController, Button.kBack.value).whenPressed(new Burp())
      .whenReleased(new StopIntakeAndConveyor());

    new POVButton(secondaryController, 90).whenPressed(new DeployIntake());
    new POVButton(secondaryController, 270).whenPressed(new HomeTowerAndRetractIntake());
    
    new POVButton(secondaryController, 180).toggleWhenPressed(new HomeTower().andThen(new AutoVisionShoot()));
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
