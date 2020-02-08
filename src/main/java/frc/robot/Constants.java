/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Constants {
  public static final int MOTOR_LEFT_1_ID = 1;
  public static final int MOTOR_LEFT_2_ID = 2;
  public static final int MOTOR_RIGHT_1_ID = 3;
  public static final int MOTOR_RIGHT_2_ID = 4;

  // robot constants

  public static final double INVERSE_KINEMATICS_TURN_EPSILON = 0.05;

  // robot dimensions

  // TODO: set actual values!
  public static final double DRIVE_WHEEL_TRACK_WIDTH = 70;
  public static final double TRACK_SCRUB_FACTOR = 0.5;

  // Drivetrain pid values
  public static final double DRIVETRAIN_P = 1;
  public static final double DRIVETRAIN_I = 1;
  public static final double DRIVETRAIN_D = 1;

  public static final double DRIVETRAIN_POSITION_TOLERANCE = 1;
}