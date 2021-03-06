/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Servo;

public class ShooterServo extends SubsystemBase {
  /**
   * This system controls the servo for the shooter
   */

  private final Servo gateServo;

  public ShooterServo() {
    gateServo = new Servo(6);
    openGap();
  }

  public void closeGap(){
    gateServo.set(1);
  }

  public void openGap(){
    gateServo.set(0.5);
  }

  @Override
  public void periodic() {
  }
}
