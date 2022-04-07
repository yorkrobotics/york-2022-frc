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
    public static final int CONTROLLER_PORT_SECONDARY = 1;

    public static final double MAX_OPENLOOP_SPEED = 0.8;

    public static final double DRIVE_MAX_RPM = 5400;


    public static final double GEAR_RATIO_LOW = 16.364;
    public static final double GEAR_RATIO_HIGH = 5.601;
      //gear ratio: 16.364, 5.601

    public static final double TRACK_WIDTH = 0.617; //The distance between left wheels and right wheels
    public static final double WHEEL_DIAMETER = 0.1524;
    
    
    public static final double kMaxVoltage = 12;
    public static final double kMaxVelocity = 1; // m/s
    public static final double kMaxAcceleration = 1; // m/s^2


    public static final double kP_VELOCITY_DRIVE_LOW_GEAR = 0.000097988;
    public static final double kI_VELOCITY_DRIVE_LOW_GEAR = 0;
    public static final double kD_VELOCITY_DRIVE_LOW_GEAR = 0;

    public static final double kP_VELOCITY_DRIVE_HIGH_GEAR = 0.000097988;
    public static final double kI_VELOCITY_DRIVE_HIGH_GEAR = 0;
    public static final double kD_VELOCITY_DRIVE_HIGH_GEAR = 0;

    public static final double kP_POSITION_DRIVE_LOW_GEAR = 0.3;
    public static final double kI_POSITION_DRIVE_LOW_GEAR = 0;
    public static final double kD_POSITION_DRIVE_LOW_GEAR = 0;

    public static final double kP_POSITION_DRIVE_HIGH_GEAR = 0.3;
    public static final double kI_POSITION_DRIVE_HIGH_GEAR = 0;
    public static final double kD_POSITION_DRIVE_HIGH_GEAR = 0;

    public static final double kP_VELOCITY_SHOOTER = 0.05;
    public static final double kI_VELOCITY_SHOOTER = 0.001;
    public static final double kD_VELOCITY_SHOOTER = 0.0;
    public static final double kFF_SHOOTER = 0.011;

    public static final double kP_VISION_YAW = 0.04;
    public static final double kI_VISION_YAW = 0.04;

    public static final double kP_AUTO_LOW_GEAR = 1.2314;

    public static final double kS_LOW_GEAR = 0.20274;
    public static final double kV_LOW_GEAR = 4.2658;
    public static final double kA_LOW_GEAR = 0.63221;

    public static final double kP_AUTO_HIGH_GEAR = 3.6108;

    public static final double kS_HIGH_GEAR = 0.25126;
    public static final double kV_HIGH_GEAR = 3.6168;
    public static final double kA_HIGH_GEAR = 1.4465;

    public static final double FIELD_CENTER_X = 19.16;
    public static final double FIELD_CENTER_Y = 0;

    public static final String PATH_FOLDER = "paths/output";


    public static final double INTAKE_ROLLER_SPEED = 0.55;
    public static final double CONVEYOR_SPEED = 0.75;


    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode CLIMB_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TOWER_IDLE_MODE = IdleMode.kBrake;


    public static final float TOWER_FORWARD_LIMIT = 243;
    public static final float CLIMB_REVERSE_LIMIT = -258;
    public static final float STATIONARY_CLIMB_FORWARD_LIMIT = 209;


    public static final double CLIMB_UPPER_PRESET = -262;
    public static final double CLIMB_LOWER_PRESET = 0;

    public static final double SLEW_RATE_LIMIT_LOW_GEAR = 1.75;
    public static final double SLEW_RATE_LIMIT_HIGH_GEAR = 1.5;



    
    public final class SparkMax{
      public static final int DRIVE_LEFT_FRONT = 3;
      public static final int DRIVE_LEFT_BACK = 4;
      public static final int DRIVE_RIGHT_FRONT = 2;
      public static final int DRIVE_RIGHT_BACK = 1;
      public static final int CLIMB_LEFT_PRIMARY = 8;
      public static final int CLIMB_RIGHT_PRIMARY = 7;
      public static final int TOWER_ACTUATOR_LEFT = 6;
      public static final int TOWER_ACTUATOR_RIGHT = 5;
      public static final int STATIONARY_CLIMB_LEFT = 13;
      public static final int STATIONARY_CLIMB_RIGHT = 14;
    }

    public final class VictorSPX{
      public static final int SHOOTER_TOP = 11;
      public static final int SHOOTER_BOTTOM = 9;
      public static final int SHOOTER_CONVEYOR = 10;
      public static final int INTAKE_ROLLER = 12;
    }

    public final class DIO {
      public static final int SHOOTER_ENCODER_TOP = 0;
      public static final int SHOOTER_ENCODER_BOTTOM = 1;
    }

    public final class PCM{
      public static final int DRIVE_GEAR_SHIFT_FORWARD = 4;
      public static final int DRIVE_GEAR_SHIFT_REVERSE = 3;
      public static final int INTAKE_FORWARD = 5;
      public static final int INTAKE_REVERSE = 2;
    }


}
