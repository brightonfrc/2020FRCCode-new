/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helperClasses.ColorSensor;

public class WheelOfFortune extends SubsystemBase {
  /**
   * Combines a coloursensor and a motor to turn the colourwheel
   */

  public VictorSPX rotatingMotor;
  public ColorSensor colorSensor;


  public WheelOfFortune() {
    rotatingMotor = new VictorSPX(Constants.WHEEL_OF_FORTUNE_MOTOR_ID);
    colorSensor = new ColorSensor(Constants.COLOR_SENSOR_I2C_PORT);
  }

  public void initDefaultCommand() {
    System.out.println("Rotation control");
  }

  @Override
  public void periodic() {
    
  }
}
