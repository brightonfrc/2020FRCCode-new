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
import frc.robot.customDatatypes.DriveSignal;

public class DriverControls extends CommandBase {
  /**
   * Creates a new DriverControls.
   * Requires the drivetrain
   * Allows to control the robot drivetrain manually
   */

  public DriverControls() {
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
    double turn = Robot.oi.twistAxis;

    // reverse turn when going backwards
    if(Robot.oi.yAxis > 0){
      turn = -turn;
    }

    double speedMultiplier = Constants.MANUAL_DRIVE_MULTIPLIER;

    // buttons 11 or 12 can be pressed down to decrease the driving speed
    if(Robot.oi.stick.getRawButton(11) || Robot.oi.stick.getRawButton(12)){
      speedMultiplier = Constants.SLOW_MANUAL_DRIVE_MULTIPLIER;
    }


    Robot.driveTrain.curvatureDrive(speedMultiplier * Robot.oi.yAxis, speedMultiplier * turn);
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
    return false;
  }
}
