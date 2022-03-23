// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  private static final Conveyor mConveyor = new Conveyor();
  public static Conveyor getInstance() {return mConveyor;}

  private VictorSPX mConveyorMotor;

  /** Creates a new Conveyor. */
  public Conveyor() {
    mConveyorMotor = new VictorSPX(Constants.VictorSPX.SHOOTER_CONVEYOR);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Run the conveyor
   * @param speed speed between (-1, 1)
   */
  public void runConveyor(double speed){
    mConveyorMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  /**
   * Stop conveyor
   */
  public void stopConveyor(){
    mConveyorMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }
}
