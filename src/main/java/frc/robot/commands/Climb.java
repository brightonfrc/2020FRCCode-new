/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climb extends CommandBase {
  /**
   * Creates a new Climb.
   * Requires the climber subsystem
   * Allows to control the position of the climber, as well as lock it
   */
  public Climb() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 5 and 6 are the buttons for moving the climber up and down
    double directionMultiplier;
    // if the input is incorrect(both or none of the buttons are pressed), freeze
    if(Robot.oi.stick.getRawButton(5) && Robot.oi.stick.getRawButton(6) || !(Robot.oi.stick.getRawButton(5) || Robot.oi.stick.getRawButton(6))){
      directionMultiplier = 0;
    }else if(Robot.oi.stick.getRawButton(5)){
      directionMultiplier = -1;
    }else{
      directionMultiplier = 1;
    }

    Robot.climber.setMotorSpeed(directionMultiplier * Constants.CLIMBER_MOTOR_MAX_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.climber.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
