/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

/**
 * The Constnts.java is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Constants {
  public static final int MOTOR_LEFT_1_ID = 4;
  public static final int MOTOR_LEFT_2_ID = 1;
  public static final int MOTOR_RIGHT_1_ID = 2;
  public static final int MOTOR_RIGHT_2_ID = 3;

  public static final int WHEEL_OF_FORTUNE_MOTOR_ID = 5;

  public static final I2C.Port COLOR_SENSOR_I2C_PORT = I2C.Port.kOnboard;
  
  // robot dimensions

  // TODO: set actual values!
  public static final double DRIVE_WHEEL_TRACK_WIDTH = 0.55;
  public static final double TRACK_SCRUB_FACTOR = 0.3;

  // Drvietrain
  public static final double MANUAL_TURN_THRESHOLD = 0.2;
  public static final double MANUAL_QUICK_TURN_THROTTLE_THRESHOLD = 0.2;

  // Drivetrain pid values
  public static final double DRIVETRAIN_THRUST_P = 1;
  public static final double DRIVETRAIN_THRUST_I = 1;
  public static final double DRIVETRAIN_THRUST_D = 1;

  public static final double DRIVETRAIN_ROTATION_P = 1;
  public static final double DRIVETRAIN_ROTATION_I = 1;
  public static final double DRIVETRAIN_ROTATION_D = 1;

  public static final double DRIVETRAIN_AUTONOMOUS_SPEED = 1;

  public static final double MINIMAL_DISTANCE_FOR_SHOOTING_RANGE_CHECK = 5;


  // Wheel of fortune code
  public static final double WHEEL_OF_FORTUNE_ROTATION_SPEED = 0.5; 

  // Color control rotations
  public static final int MIN_COLOR_CONTROL_ROTATIONS = 3;
  public static final int MAX_COLOR_CONTROL_ROTATIONS = 5;
  public static final int NUMBER_OF_REPETITIONS_OF_COLOR_PER_TURN = 2;

  // Colors
  public static double[] BLUE_VALUES = {0.143, 0.427, 0.429};

  public static double[] GREEN_VALUES = {0.197, 0.561, 0.240};
  public static double[] RED_VALUES = {0.561, 0.232, 0.114};
  public static double[] YELLOW_VALUES = {0.361, 0.524, 0.113};

  public static Color BLUE_TARGET = ColorMatch.makeColor(BLUE_VALUES[0], BLUE_VALUES[1], BLUE_VALUES[2]);
  public static Color GREEN_TARGET = ColorMatch.makeColor(GREEN_VALUES[0], GREEN_VALUES[1], GREEN_VALUES[2]);
  public static Color RED_TARGET = ColorMatch.makeColor(RED_VALUES[0], RED_VALUES[1], RED_VALUES[2]);
  public static Color YELLOW_TARGET = ColorMatch.makeColor(YELLOW_VALUES[0], YELLOW_VALUES[1], YELLOW_VALUES[2]);
}