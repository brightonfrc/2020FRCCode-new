/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.customDatatypes.DriveSignal;

public class DriveDistance extends CommandBase {
  /**
   * Creates a new DriveDistance.
   * Requires a drivetrain
   * Drives forward a certain distance based on time. The variables recorded are not accurate, so the command does not work. 
   * Encoders would have made the job easier
   */

  private long startTime = 0;
  private long timeToDrive = 0;

  public DriveDistance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  public void goDistance(double d){
    // // if impossible to move,
    // if(d < Constants.BREAK_AND_START_DISTANCE){
    //   throw new Error("Can not do timed movement on such short distances");
    //   return;
    // }

    // timeToDrive = 1000 * (d - Constants.BREAK_AND_START_DISTANCE) / Constants.ROBOT_SPEED;
    // Robot.driveTrain.arcadeDrive(0.3, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop the drivetrain
    Robot.driveTrain.tankDrive(DriveSignal.NEUTRAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if the time eceeds or is equal to the finish time
    return timeToDrive != 0 && System.currentTimeMillis() - startTime > timeToDrive;
  }
}
