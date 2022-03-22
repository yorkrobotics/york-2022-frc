// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

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

    public static final double TRACK_WIDTH = 0.617; //The distance between left wheels and right wheels
    public static final double WHEEL_DIAMETER = 0.1524;
    
    
    public static final double kMaxVoltage = 12;
    public static final double kMaxVelocity = 1; // m/s
    public static final double kMaxAcceleration = 1; // m/s^2

    public static final double kP_AUTO_LOW_GEAR = 5.5181;

    public static final double kS_LOW_GEAR = 0.20874;
    public static final double kV_LOW_GEAR = 4.3531;
    public static final double kA_LOW_GEAR = 0.53187;

    

    public static final double FIELD_CENTER_X = 19.16;
    public static final double FIELD_CENTER_Y = 0;

    public static final String PATH_FOLDER = "paths/output";


    public static final double INTAKE_ROLLER_SPEED = 0.4;
    public static final double CONVEYOR_SPEED = 0.6;


    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kCoast;
    public static final IdleMode CLIMB_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TOWER_IDLE_MODE = IdleMode.kBrake;




    
    public final class SparkMax{
      public static final int DRIVE_LEFT_FRONT = 3;
      public static final int DRIVE_LEFT_BACK = 4;
      public static final int DRIVE_RIGHT_FRONT = 2;
      public static final int DRIVE_RIGHT_BACK = 1;
      public static final int CLIMB_LEFT_PRIMARY = 8;
      public static final int CLIMB_RIGHT_PRIMARY = 7;
      public static final int TOWER_ACTUATOR_LEFT = 6;
      public static final int TOWER_ACTUATOR_RIGHT = 5;
    }

    public final class VictorSPX{
      public static final int SHOOTER_TOP = 11;
      public static final int SHOOTER_BOTTOM = 9;
      public static final int SHOOTER_CONVEYOR = 10;
      public static final int INTAKE_ROLLER = 12;
    }

    public final class PCM{
      public static final int DRIVE_GEAR_SHIFT_FORWARD = 4;
      public static final int DRIVE_GEAR_SHIFT_REVERSE = 3;
      public static final int INTAKE_FORWARD = 5;
      public static final int INTAKE_REVERSE = 2;
    }
}
