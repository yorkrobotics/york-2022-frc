// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {

  private static final ColorSensor mCS = new ColorSensor();
  public static ColorSensor getInstance() {return mCS;}

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 mColorSensor = new ColorSensorV3(i2cPort);

  private final ColorMatch mColorMatcher = new ColorMatch();

  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);

  /** Creates a new ColorSensor. */
  public ColorSensor() {
    mColorMatcher.addColorMatch(kRedTarget);
    mColorMatcher.addColorMatch(kBlueTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor =  mColorSensor.getColor();

    String colorString;
    ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget){
      colorString = "blue";
    }
    if (match.color == kRedTarget){
      colorString = "red";
    }
    else {
      colorString = "unknown";
    }

    SmartDashboard.putNumber("red", detectedColor.red);
    SmartDashboard.putNumber("blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
}
