/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.WheelOfFortune;

public class PositionControl extends CommandBase {
  /**
   * Creates a new PositionControl.
   */
  public PositionControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.wheelOfFortune);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Position control color index: " + getCurrentIndex());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void debugColorPredictions(){
    System.out.println();
  }

  private int getFRCSensorColorIndex(int currentIndex){
    return (currentIndex + 2) % 4;
  }

  private int getCurrentIndex(){
    // get the current colour
    ColorMatchResult color = Robot.wheelOfFortune.colorSensor.getColourLabel();
    for(int i = 0; i < 4; i++){
      if(color.color == Constants.CLOCKWISE_COLORS_ON_THE_WHEEL[i]){
        return i;
      }
    }

    return -1;
  }
}
