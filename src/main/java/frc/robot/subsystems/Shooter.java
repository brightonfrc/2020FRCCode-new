/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Shooter extends SubsystemBase {

  private final VictorSPX m_motorLeft;
  private final VictorSPX m_motorRight;

  public Shooter() {
    m_motorLeft = new VictorSPX(Constants.SHOOTER_LEFT_MOTOR_ID);
    m_motorRight = new VictorSPX(Constants.SHOOTER_RIGHT_MOTOR_ID);

    m_motorRight.follow(m_motorLeft);
    m_motorRight.setInverted(true);

    // pre-spin the motors
    // m_motorLeft.set(ControlMode.PercentOutput, Constants.SHOOTER_MOTORS_SPEED);
  }

  @Override
  public void periodic() {
  }
}

