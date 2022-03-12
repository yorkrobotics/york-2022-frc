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

    public static final int CONTROLLER_PORT = 0;


    public static final double MAX_OPENLOOP_SPEED = 0.5;

    public static final double DRIVE_MAX_RPM = 5400;


    public static final double GEAR_RATIO_LOW = 16.364;
    public static final double GEAR_RATIO_HIGH = 5.601;
      //gear ratio: 16.364, 5.601

    public static final double TRACK_WIDTH = 0.655; //The distance between left wheels and right wheels
    
    public static final double kMaxVoltage = 12;
    public static final double kMaxVelocity = 1; // m/s
    public static final double kMaxAcceleration = 1; // m/s^2

    public static final double kP_AUTO = 5.7673;

    public static final double FIELD_CENTER_X = 19.16;
    public static final double FIELD_CENTER_Y = 0;

    public static final String PATH_FOLDER = "paths/output";
    
    public final class CAN{
      public static final int DRIVE_LEFT_FRONT = 1;
      public static final int DRIVE_LEFT_BACK = 2;
      public static final int DRIVE_RIGHT_FRONT = 4;
      public static final int DRIVE_RIGHT_BACK = 3;
      public static final int SHOOTER_LEFT_PRIMARY = 0;
      public static final int SHOOTER_RIGHT_PRIMARY = 0;
    }

    public final class PWM{
      public static final int SHOOTER_TOP = 0;
      public static final int SHOOTER_BOTTOM = 1;
      public static final int SHOOTER_CONVEYOR = 0;
      public static final int INTAKE_ROLLER = 0;
    }

    public final class PCM{
      public static final int DRIVE_GEAR_SHIFT_FORWARD = 6;
      public static final int DRIVE_GEAR_SHIFT_REVERSE = 1;
      public static final int INTAKE_FORWARD = 0;
      public static final int INTAKE_REVERSE = 0;
    }
}
