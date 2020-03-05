/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helperClasses;

import frc.robot.Constants;

public class ComputerVision  {
  /**
   * Creates a new computerVision.
   */

  private double distance_d, side_d, distance_d_adjusted, yaw_angle_adjusted;

  // constants:

  final static int WIDTH = 0;
  final static int HEIGHT = 1;

  // outer port target dimensions (cm)
  //  3ft 3 1/4 in by 1 ft. 5 in
  final static double[] outerPortTargetDims = {99.695, 43.18};

  // loading bay target dimensions
  // 7 in by 11 in
  // 11 in from the ground
  final static double[] loadingBayTargetDims = {17.78, 27.94};
  
  public ComputerVision() {
    // Put methods for controlling this subsystem
  // here. Call these from Commands.
  }

  // TODO: change this
  public double getAngleToTarget(){
    return 0d;
  }

  public double getDistanceToTarget(){
    return 7d;
  }

  // TODO: make sure to call this before yawAdjusted
  public double getdAdjusted(double pitch_angle, double yaw_angle){
    double distance_d = Constants.DELTA_H / Math.tan(Math.toRadians(Constants.ANGLE_I + pitch_angle)); // This is the distance from
                                                                                           // the CAMERA to the centre
                                                                                           // of target
    double side_d = distance_d / Math.tan(Math.toRadians(90-yaw_angle));
    double distance_d_adjusted = Math.sqrt(Math.pow(side_d,2) + Math.pow(distance_d + Constants.LENGTH_E_C,2));                                                                                

    // double distance_d_adjusted = Math.sqrt((Math.pow(distance_d, 2)) + (Math.pow(Constants.LENGTH_E_C, 2))
    //     - ((2 * distance_d * Constants.LENGTH_E_C) * Math.cos(Math.toRadians(180 - yaw_angle)))); // This is the
    //                                                                                               // distance from the
    //                                                                                               // CENTRE OF ROBOT (E)
    //                                                                                               // to centre of target

    this.distance_d = distance_d;
    this.side_d = side_d;
    this.distance_d_adjusted = distance_d_adjusted;

    return distance_d_adjusted;
  }

  public double getYawAdjusted(){
    double yaw_angle_adjusted = Math.toDegrees(Math.atan((distance_d + Constants.LENGTH_E_C) / side_d));

    this.yaw_angle_adjusted = yaw_angle_adjusted;

    return yaw_angle_adjusted;
  }
}
