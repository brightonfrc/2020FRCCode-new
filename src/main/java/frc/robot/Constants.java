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
  public static final int SHOOTER_LEFT = 5;
  public static final int SHOOTER_RIGHT = 6;


  public static final int WHEEL_OF_FORTUNE_MOTOR_ID = 7;

  public static final int SHOOTER_LEFT_MOTOR_ID = 5;
  public static final int SHOOTER_RIGHT_MOTOR_ID = 6;

  // robot constants

  // constants for automatic shooting w/CV:
  // accounting for component displacement: all in metres (NOTE: c = camera; e = centre of robot; t = tip of shooter)
  public static final double LENGTH_E_C = 0.4; 

  // TODO: replace "69" with actual values! 
  public static final int CAMERA_RESOLUTION_X = 640;
  public static final int CAMERA_RESOLUTION_Y = 480;
  public static final double HEIGHT_OF_CAMERA_FROM_FLOOR = 0.87; // Meters
  public static final double PIXEL_DEGREE_HORIZONTAL_CONVERT = 0.09375;
  public static final double PIXEL_DEGREE_VERTICAL_CONVERT = 0.06996; //Both converts in degrees per pixel
  // public static final double SHOOTER_V = 69; // FIND THIS OUT, wont be needed in calculations, just needs to stay constant (velocity)
  public static final double DELTA_H = FieldMap.TARGET_HEIGHT - HEIGHT_OF_CAMERA_FROM_FLOOR;
  public static final double DISTANCE_I = 3.88 - LENGTH_E_C; // Replace this "69" with any realistic distance, from which the camera's image's centre is at the centre of target
  public static final double ANGLE_I = Math.atan(DELTA_H/DISTANCE_I); 
  public static final double DISTANCE_D = 2.43; // FIND THIS OUT (THIS IS CRUCIAL) it is the distance from centre of robot to target centre where shooter scores
  // for 0.5 power, 2.43
  public static final double MAXIMUM_DRIVE_VELOCITY = 69; // --TEST THIS ON MONDAY-- //
  // public static final double TIME_CUTOFF_PROPORTION = 69; // --TEST THIS ON MONDAY-- // or use value 0.05 as a guesstimate
  // public static final long TIME_DELAY_BEFORE_RESCAN = 69; // --TEST THIS ON MONDAY-- // or use value 100 as a guesstimate (this time is in milliseconds)

  // Note: define more displacement constants if needed above

  public static final double SHOOTER_MOTORS_SPEED = 0.5;

  public static final int X_TOLERANCE = 3; // --TEST THIS ON MONDAY-- // or use value 3 as a guesstimate (THESE VALUES ARE IN PIXELS)
  public static final int Y_TOLERANCE = 1; // --TEST THIS ON MONDAY-- // or use value 1 as a guesstimate

  // robot constants
  
  public static final I2C.Port COLOR_SENSOR_I2C_PORT = I2C.Port.kOnboard;
  
  // robot dimensions

  // TODO: set actual values!
  public static final double DRIVE_WHEEL_TRACK_WIDTH = 0.55;
  public static final double TRACK_SCRUB_FACTOR = 0.5;

  // Drvietrain
  public static final double MANUAL_TURN_THRESHOLD = 0.2;
  public static final double MANUAL_QUICK_TURN_THROTTLE_THRESHOLD = 0.2;

  public static final double MANUAL_DRIVE_MULTIPLIER = 0.7;
  public static final double SLOW_MANUAL_DRIVE_MULTIPLIER = 0.3;

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
  public static final double WHEEL_OF_FORTUNE_ROTATION_SPEED = 0.2; 
  public static final double MANUAL_CONTROL_WHEEL_OF_FORTUNE_ROTATION_SPEED = 0.3; 

  public static final long TIME_TO_WAIT_FOR_WHEEL_TO_SETTLE = 500;

  // Color control rotations
  public static final int MIN_COLOR_CONTROL_ROTATIONS = 3;// 3
  public static final int MAX_COLOR_CONTROL_ROTATIONS = 5;
  public static final int NUMBER_OF_REPETITIONS_OF_COLOR_PER_TURN = 2; //2

  // Colors
  public static double[] BLUE_VALUES = {0.143, 0.427, 0.429};

  public static double[] GREEN_VALUES = {0.197, 0.561, 0.240};
  public static double[] RED_VALUES = {0.561, 0.232, 0.114};
  public static double[] YELLOW_VALUES = {0.361, 0.524, 0.113};

  public static Color BLUE_TARGET = ColorMatch.makeColor(BLUE_VALUES[0], BLUE_VALUES[1], BLUE_VALUES[2]);
  public static Color GREEN_TARGET = ColorMatch.makeColor(GREEN_VALUES[0], GREEN_VALUES[1], GREEN_VALUES[2]);
  public static Color RED_TARGET = ColorMatch.makeColor(RED_VALUES[0], RED_VALUES[1], RED_VALUES[2]);
  public static Color YELLOW_TARGET = ColorMatch.makeColor(YELLOW_VALUES[0], YELLOW_VALUES[1], YELLOW_VALUES[2]);

  public static final Color[] CLOCKWISE_COLORS_ON_THE_WHEEL = {BLUE_TARGET, GREEN_TARGET, RED_TARGET, YELLOW_TARGET}; 
}
