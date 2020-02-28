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

  private final VictorSPX m_shooterLeft;
  private final VictorSPX m_shooterRight;

  public Shooter() {
    this.m_shooterLeft = new VictorSPX(Constants.SHOOTER_LEFT);
    this.m_shooterRight = new VictorSPX(Constants.SHOOTER_RIGHT);
  }

  @Override
  public void periodic() {
    this.m_shooterLeft.set(ControlMode.PercentOutput, 0.5);
    this.m_shooterRight.set(ControlMode.PercentOutput, -0.5);
  }
}
