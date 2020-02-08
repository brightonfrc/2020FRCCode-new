/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Shooter extends SubsystemBase {
   private VictorSPX shooterMotor1;

  public Shooter() {
    shooterMotor1 = new VictorSPX(1);
  }

  @Override
  public void periodic() {
    shooterMotor1.set(ControlMode.PercentOutput, 1);
  }
}

