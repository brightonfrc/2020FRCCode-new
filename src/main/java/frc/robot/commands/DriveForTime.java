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


public class DriveForTime extends CommandBase {
  /**
   * Creates a new SpeedTest.
   * This command requires a drivetrain
   * It is used to drive forward for a certain amount of time
   */

  protected double power;
  protected double time;
  protected double endTime;

  /**
   * Time is in milliseconds
   */
  public DriveForTime(double power, double time) {
    this.power = power;
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    long startTime = System.currentTimeMillis();
    this.endTime = startTime + this.time;  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.driveTrain.tankDrive(new DriveSignal(this.power, this.power));
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
