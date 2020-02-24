/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

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

  // constants for automatic shooting w/CV:
  // TODO: replace "69" with actual values! 
  public static final double PIXEL_DEGREE_HORIZONTAL_CONVERT = 0.09375;
  public static final double PIXEL_DEGREE_VERTICAL_CONVERT = 0.06996; //Both converts in degrees per pixel
  public static final double SHOOTER_V = 69; // FIND THIS OUT, wont be needed in calculations, just needs to stay constant (velocity)
  public static final double DELTA_H = FieldMap.TARGET_HEIGHT - 69; // FIND OUT THE HEIGHT OF CAMERA FROM THE FLOOR, this is used to calculate d
  public static final double ANGLE_I = Math.atan(DELTA_H/69); // Replace this "69" with any realistic distance, from which the camera's image's centre is at the centre of target
  public static final double DISTANCE_D = 69; // FIND THIS OUT (THIS IS CRUCIAL) it is the distance from centre of robot to target centre where shooter scores
  public static final double MAXIMUM_DRIVE_VELOCITY = 69; // --TEST THIS ON MONDAY-- //
  public static final double TIME_CUTOFF_PROPORTION = 69; // --TEST THIS ON MONDAY-- // or use value 0.05 as a guesstimate
  public static final long TIME_DELAY_BEFORE_RESCAN = 69; // --TEST THIS ON MONDAY-- // or use value 100 as a guesstimate (this time is in milliseconds)
  // accounting for component displacement: all in metres (NOTE: c = camera; e = centre of robot; t = tip of shooter)
  public static final double LENGTH_E_C = 69; 
  // Note: define more displacement constants if needed above

  
  // robot dimensions

  // TODO: set actual values!
  public static final double DRIVE_WHEEL_TRACK_WIDTH = 70;
  public static final double TRACK_SCRUB_FACTOR = 0.5;

  // Drivetrain pid values
  public static final double DRIVETRAIN_P = 1;
  public static final double DRIVETRAIN_I = 1;
  public static final double DRIVETRAIN_D = 1;

  public static final double DRIVETRAIN_POSITION_TOLERANCE = 1;
  public static final int X_TOLERANCE = 69; // --TEST THIS ON MONDAY-- // or use value 3 as a guesstimate
  public static final int Y_TOLERANCE = 69; // --TEST THIS ON MONDAY-- // or use value 1 as a guesstimate




  // Colours
  public static double[] BLUE_VALUES = {0.143, 0.427, 0.429};
  public static double[] GREEN_VALUES = {0.197, 0.561, 0.240};
  public static double[] RED_VALUES = {0.561, 0.232, 0.114};
  public static double[] YELLOW_VALUES = {0.361, 0.524, 0.113};

  public static Color BLUE_TARGET = ColorMatch.makeColor(BLUE_VALUES[0], BLUE_VALUES[1], BLUE_VALUES[2]);
  public static Color GREEN_TARGET = ColorMatch.makeColor(GREEN_VALUES[0], GREEN_VALUES[1], GREEN_VALUES[2]);
  public static Color RED_TARGET = ColorMatch.makeColor(RED_VALUES[0], RED_VALUES[1], RED_VALUES[2]);
  public static Color YELLOW_TARGET = ColorMatch.makeColor(YELLOW_VALUES[0], YELLOW_VALUES[1], YELLOW_VALUES[2]);
}