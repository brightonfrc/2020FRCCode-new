/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  private VictorSPX m_motorLeft, m_motorRight;

  public Climber() {
    m_motorRight = new VictorSPX(Constants.CLIMBER_MOTOR_RIGHT_ID);
    m_motorLeft = new VictorSPX(Constants.CLIMBER_MOTOR_LEFT_ID);
  }

  public void setMotorSpeed(double speed){
    m_motorLeft.set(ControlMode.PercentOutput, speed);
    m_motorRight.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
