// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;

public class DeployIntake extends InstantCommand {

  private Intake mIntake;
  private Tower mTower;
  
  /** Creates a new DeployIntake. */
  public DeployIntake(Intake intake, Tower tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    mTower = tower;
    addRequirements(mIntake, mTower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTower.whenIntakeDeployed();
    mIntake.deploy();
  }

}
