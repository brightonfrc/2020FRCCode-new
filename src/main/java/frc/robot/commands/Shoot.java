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

public class Shoot extends CommandBase {
  /**
   * Creates a new DriverControls.
   * Requires the shooter and shooter servo subsystems
   * The subsystem shoots when a button is pressed
   */

  private long startTime;
  private boolean isServoLifted;

  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.shooterServo);
    addRequirements(Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset the time for shooting
    startTime = System.currentTimeMillis();
    isServoLifted = false;

    // spins up the motors
    Robot.shooter.startMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // releases the servo when the motors have spun up
    if(!isServoLifted && System.currentTimeMillis() - startTime > Constants.SHOOTER_MILLIS_TO_RELEASE){
      isServoLifted = true;
      Robot.shooterServo.openGap();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    // stops the motors and closes the servo
    Robot.shooterServo.closeGap();
    Robot.shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // finishes the command by times
    return System.currentTimeMillis() - startTime > Constants.SHOOTER_COMMAND_TIME_TO_FINISH;
  }
}
