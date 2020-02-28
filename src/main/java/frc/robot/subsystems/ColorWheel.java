/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import frc.robot.helperClasses.ColorSensor;
import edu.wpi.first.wpilibj.I2C.Port;



/**
 * Add your docs here.
 */
public class ColorWheel extends SubsystemBase {
  // private final VictorSPX colorMotor;
  // private final ColorSensor colorSensor;

  // create groups for motors
  // private final SpeedControllerGroup m_leftMotors;
  // private final SpeedControllerGroup m_rightMotors;

  // private final DifferentialDrive m_differentialDrive;

  public ColorWheel() {
    // colorMotor = new VictorSPX(Constants.MOTOR_LEFT_1_ID);
    // colorSensor = new ColorSensor(new Port("0x56"))
  }

  @Override
  public void periodic() {
    // ColorSensor.
  }


}
