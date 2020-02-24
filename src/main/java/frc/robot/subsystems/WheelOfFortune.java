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
import frc.robot.commands.WheelOfFortuneCommand;
import frc.robot.helperClasses.ColorSensor;

public class WheelOfFortune extends SubsystemBase {
  /**
   * Creates a new wheelOfFortune.
   */

  public VictorSPX rotatingMotor;
  public ColorSensor colorSensor;

  private WheelOfFortuneCommand wheelOfFortuneCommand;

  public WheelOfFortune() {
    rotatingMotor = new VictorSPX(Constants.WHEEL_OF_FORTUNE_MOTOR_ID);
    colorSensor = new ColorSensor(Constants.COLOR_SENSOR_I2C_PORT);
  }

  public void initDefaultCommand() {
    System.out.println("Added rotation control");
  }

  public void startRotationControl(){
    wheelOfFortuneCommand.setState(WheelOfFortuneCommand.ROTATION_CONTROL);

    //System.out.println("Start rotation control call");

    // set the rotation control command
    //CommandScheduler.getInstance().schedule(m_rotationControl);
  }

  public void startPositionControl(){
    wheelOfFortuneCommand.setState(WheelOfFortuneCommand.POSITION_CONTROL);

    // stop the rotation control
    //CommandScheduler.getInstance().cancel(m_rotationControl);
  }

  @Override
  public void periodic() {
    //System.out.println(getCurrentCommand());
    // This method will be called once per scheduler run
  }



  // getters and setters
  public void setWheelCommand(WheelOfFortuneCommand command){
    wheelOfFortuneCommand = command;
  }

  public WheelOfFortuneCommand getWheelCommand(){
    return wheelOfFortuneCommand;
  }
}
