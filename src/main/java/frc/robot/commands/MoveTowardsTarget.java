/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.customDatatypes.DriveSignal;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class MoveTowardsTarget extends PIDCommand {
  /**
   * Creates a new MoveTowardsTarget.
   */
  public MoveTowardsTarget() {
    super(
        // The controller that the command will use
        new PIDController(Constants.DRIVETRAIN_ROTATION_P, Constants.DRIVETRAIN_ROTATION_I, Constants.DRIVETRAIN_ROTATION_D),
        // This should return the measurement
        Robot.computerVision::getAngleToTarget,
        // This should return the setpoint (can also be a constant)
        0d,
        // This uses the output
        output -> {
          // Use the output here
          Robot.driveTrain.curvatureDrive(Constants.DRIVETRAIN_AUTONOMOUS_SPEED, -output);
        });

    // TODO: set the ranges


    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    addRequirements(Robot.driveTrain);
  }

  // Called once after isFinished returns true
  // end is overriden by PIDCommand, so this is a new one
  protected void endOfCommand() {
    Robot.driveTrain.tankDrive(DriveSignal.NEUTRAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Robot.computerVision.getDistanceToTarget() < Constants.MINIMAL_DISTANCE_FOR_SHOOTING_RANGE_CHECK){
      // TODO: check if the shooter can shoot now
      if(true){
        endOfCommand();
        return true;
      }
    }
    return false;
  }
}
