/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.customDatatypes.DriveSignal;


public class TimeDriveToGoal extends CommandBase {
  /**
   * Creates a new SpeedTest.
   * Requires the drivetrain subsystem
   * Drives forward at a set power percentage while recording the time. It is used to record the speed values for DriveDistance
   */

  protected double power;
  protected double distance;
  protected double time;
  protected double endTime;

  /**
   * Time is in milliseconds
   */
  public TimeDriveToGoal() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.power = 0.2;

    double pitch = Robot.pitchEntry.getDouble(0.0);

    this.distance = Robot.computerVision.getD(pitch) - Constants.DISTANCE_D;

    this.time = (this.distance - Constants.SLIP_DISTANCE)/Constants.MAX_SPEED;
    long startTime = System.currentTimeMillis();

    this.endTime = startTime + this.time;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.driveTrain.tankDrive(this.power, this.power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    // stop the drivetrain
    Robot.driveTrain.tankDrive(DriveSignal.NEUTRAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= this.endTime;
  }
}
