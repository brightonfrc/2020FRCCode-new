/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.Constants;

public class DriveToGoal extends PIDCommand {
    /**
     * Turns to robot to the specified angle.
     *
     * @param targetAngleDegrees The angle to turn to
     * @param drive              The drive subsystem to use
     */

    public DriveToGoal() {
      super(
          new PIDController(Constants.DRIVETRAIN_ROTATION_P, Constants.DRIVETRAIN_ROTATION_I, Constants.DRIVETRAIN_ROTATION_D),
          // Close loop on heading
          DriveToGoal::getError,
          // Set reference to target
          0,
          // Pipe output to turn robot
          output -> Robot.driveTrain.arcadeDrive(-output*0.3, 0),
          // Require the drive
          Robot.driveTrain);

      getController().setTolerance(3, 0.4);
    }

    @Override
    public void initialize(){
    }

    public static double getError(){
      double pitch = Robot.pitchEntry.getDouble(0.0);
      double yaw = Robot.yawEntry.getDouble(0.0);

      System.out.println(Constants.DISTANCE_D - Robot.computerVision.getdAdjusted(pitch, yaw));
      return Constants.DISTANCE_D - Robot.computerVision.getdAdjusted(pitch, yaw);
    }
  
    @Override
    public boolean isFinished() {
      System.out.println(getController().getSetpoint());
      return getController().atSetpoint();
    }
  }
