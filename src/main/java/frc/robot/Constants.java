// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // PWM wiring ports
    public static final int LEFT_FRONT = 9;
    public static final int RIGHT_FRONT = 8;
    public static final int LEFT_BACK = 0;
    public static final int RIGHT_BACK = 0;

    // xbox controller
    public static final int X_BOX_Y_AXIS = 1;
    public static final int X_BOX_X_AXIS = 0;
    public static final int X_BOX_LEFT_TRIGGER = 2;
    public static final int X_BOX_RIGHT_TRIGGER = 3;

    public static final int CONTROLLER_NUMBER = 0;


    public static final double DRIVETRAIN_SPEED = 0.5;

    public static final double DRIVE_FORWARD_TIME = 3.0;
    public static final double AUTONOMOUS_SPEED = 0.5;
    
    public static final String DRIVE_MODE = "T";    //"J" = drive with joysticks, "T" = drive with triggers


    public static final double DRIVE_KP = 0;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;
    public static final double TARGET_DISTANCE = 3.0;
    public static final double GEAR_RATIO = 0;  //gear ratio: 16.364, 5.601

    
}
