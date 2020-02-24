/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ComputerVision extends SubsystemBase {
  /**
   * Creates a new computerVision.
   */
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
