// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Joystick Controller IDs, DRIVER = 0, OPERATOR = 1
  public static final int DRIVER = 0;
  public static final int OPERATOR = 1;

  // Joystick Analog Axis/Stick //
  public static final int STICK_LEFT_X = 0;
  public static final int STICK_LEFT_Y = 1;
  public static final int TRIGGER_LEFT = 2;
  public static final int TRIGGER_RIGHT = 3;
  public static final int STICK_RIGHT_X = 4;
  public static final int STICK_RIGHT_Y = 5;

  // Joystick Buttons //
  public static final int BTN_A = 1;
  public static final int BTN_B = 2;
  public static final int BTN_X = 3;
  public static final int BTN_Y = 4;
  public static final int BUMPER_LEFT = 5;
  public static final int BUMPER_RIGHT = 6;
  public static final int BTN_BACK = 7;
  public static final int BTN_START = 8;
  public static final int BTN_STICK_LEFT = 9;
  public static final int BTN_STICK_RIGHT = 10;

  // Extreme 3D Pro Analog //
  public static final int STICK_X = 0;
  public static final int STICK_Y = 1;
  public static final int STICK_Z = 2;
  public static final int THROTTLE = 3;
  
  // Extreme 3D Pro Buttons
  public static final int BTN_TRIGGER = 1;
  public static final int BTN_THUMB = 2;
  public static final int BTN_3 = 3;
  public static final int BTN_4 = 4;
  public static final int BTN_5 = 5;
  public static final int BTN_6 = 6;
  public static final int BTN_7 = 7;
  public static final int BTN_8 = 8;
  public static final int BTN_9 = 9;
  public static final int BTN_10 = 10;
  public static final int BTN_11 = 11;
  public static final int BTN_12 = 12;

  // Motor ID Constants //

  // Drivetrain //
  public static final int DRIVETRAIN_PIGEON_ID = 0;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(238.97);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 4;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(140.62);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(325.63);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 1;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(266.48);

  // Arm //

  // Intake //

  // Robot Measurement Constants //
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.2794;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.2794;
}
