/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ComputerVision extends SubsystemBase {
  /**
   * Creates a new computerVision.
   */

  private double distance_d, distance_d_adjusted, yaw_angle_adjusted;

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

  public double getdAdjusted(double pitch_angle, double yaw_angle){
    double distance_d = Math.sin(Math.toRadians(90 - Constants.ANGLE_I))
        * (Constants.DELTA_H / Math.sin(Math.toRadians(Constants.ANGLE_I + pitch_angle))); // This is the distance from
                                                                                           // the CAMERA to the centre
                                                                                           // of target
    double distance_d_adjusted = Math.sqrt((Math.pow(distance_d, 2)) + (Math.pow(Constants.LENGTH_E_C, 2))
        - ((2 * distance_d * Constants.LENGTH_E_C) * Math.cos(Math.toRadians(180 - yaw_angle)))); // This is the
                                                                                                  // distance from the
                                                                                                  // CENTRE OF ROBOT (E)
                                                                                                  // to centre of target
    this.distance_d_adjusted = distance_d_adjusted;
    this.distance_d = distance_d;
    return distance_d_adjusted;
  }

  public double getYawAdjusted(double yaw_angle){
    double yaw_angle_adjusted = distance_d * (distance_d_adjusted / (180 - yaw_angle));
    this.yaw_angle_adjusted = yaw_angle_adjusted;
    return yaw_angle_adjusted;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
