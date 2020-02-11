/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.RotationControl;
import frc.robot.helperClasses.ColorSensor;

public class WheelOfFortune extends SubsystemBase {
  /**
   * Creates a new wheelOfFortune.
   */

  public VictorSPX rotatingMotor;
  public ColorSensor colorSensor;

  public static int IDLE = 0;
  public static int ROTATION_CONTROL = 1;
  public static int POSITION_CONTROL = 2;

  public int state = IDLE;

  private RotationControl m_rotationControl;

  public WheelOfFortune() {
    rotatingMotor = new VictorSPX(Constants.WHEEL_OF_FORTUNE_MOTOR_ID);
    colorSensor = new ColorSensor(Constants.COLOR_SENSOR_I2C_PORT);

    m_rotationControl = new RotationControl();

    // so that the scheduler knows that this subsystem exists
    CommandScheduler.getInstance().registerSubsystem();
  }

  public void startRotationControl(){
    state = ROTATION_CONTROL;

    // set the rotation control command
    CommandScheduler.getInstance().schedule(m_rotationControl);
  }

  public void startPositionControl(){
    state = POSITION_CONTROL;

    // stop the rotation control
    CommandScheduler.getInstance().cancel(m_rotationControl);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
