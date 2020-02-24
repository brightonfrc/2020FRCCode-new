/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.helperClasses.ColorSensor;

public class WheelOfFortuneCommand extends CommandBase {
  /**
   * Creates a new RotationControl.
   */

  private ColorMatchResult initialColorReading;
  private ColorMatchResult previousColorReading;
  private int numOfTimesInitialColorWasSeen;

  public static int IDLE = 0;
  public static int ROTATION_CONTROL = 1;
  public static int POSITION_CONTROL = 2;

  public int state = IDLE;

  public WheelOfFortuneCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.wheelOfFortune);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == ROTATION_CONTROL){
      rotationControlUpdate();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop spinning the motor
    Robot.wheelOfFortune.rotatingMotor.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // check if the number of turns exceeded the minimum, end
    if(Constants.MIN_COLOR_CONTROL_ROTATIONS * Constants.NUMBER_OF_REPETITIONS_OF_COLOR_PER_TURN <= numOfTimesInitialColorWasSeen){
      System.out.println("Finished rotating");
      return true;
    }else{
      return false;
    }
  }

  private void reset(){
    // reset
    initialColorReading = previousColorReading = Robot.wheelOfFortune.colorSensor.getColourLabel();
    numOfTimesInitialColorWasSeen = 1;

    System.out.println("Start wheel control command");

    // start spining the motor
    Robot.wheelOfFortune.rotatingMotor.set(ControlMode.PercentOutput, Constants.WHEEL_OF_FORTUNE_ROTATION_SPEED);
  }

  private void rotationControlUpdate(){
    System.out.println("Color rotation control");

    // if the color chaged and it is the one needed
    ColorMatchResult currentReading = Robot.wheelOfFortune.colorSensor.getColourLabel();

    ColorSensor.debugColorMatch(currentReading);

    if(currentReading.color != previousColorReading.color && currentReading == initialColorReading){
      // increase the counter by 1
      numOfTimesInitialColorWasSeen++;
    }

    previousColorReading = currentReading;
  }



  // setters and getters
  public void setState(int newState){
    if(state != newState){
      state = newState;
      reset();
    }
  }
}
