// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Burp extends InstantCommand {
  private Intake mIntake = Intake.getInstance();
  private Conveyor mConveyor = Conveyor.getInstance();
  public Burp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntake, mConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mIntake.isDeployed()){
      mIntake.runRoller(Constants.INTAKE_ROLLER_SPEED);
    }
    mConveyor.runConveyor(-Constants.CONVEYOR_SPEED);
  }
}
