/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ManualFortuneWheelControl extends CommandBase {
  /**
   * Creates a new ManualFortuneWheelControl.
   * Requires the wheel of fortune subsystem
   * Allows to manually control the speed and the direction of the rotation of the wheel
   */
  public ManualFortuneWheelControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.wheelOfFortune);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // stop
    Robot.wheelOfFortune.rotatingMotor.set(ControlMode.PercentOutput, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // rotate at the speed set by the joystick
    Robot.wheelOfFortune.rotatingMotor.set(ControlMode.PercentOutput, Robot.oi.xAxis*Constants.MANUAL_CONTROL_WHEEL_OF_FORTUNE_ROTATION_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop
    Robot.wheelOfFortune.rotatingMotor.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
