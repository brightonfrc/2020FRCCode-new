/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.Constants;

public class TurnToAngle extends PIDCommand {
    /**
     * Turns to robot to the specified angle.
     * Requires the drivetrain
     * Turns to a specified angle using the PID
     * 
     * @param targetAngleDegrees The angle to turn to
     * @param drive              The drive subsystem to use
     */

    private static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    public TurnToAngle(double angle) {
      super(
          new PIDController(Constants.DRIVETRAIN_ROTATION_P, Constants.DRIVETRAIN_ROTATION_I, Constants.DRIVETRAIN_ROTATION_D),
          // Close loop on heading
          TurnToAngle::getAngle,
          // Set reference to target
          angle,
          // Pipe output to turn robot
          output -> Robot.driveTrain.arcadeDrive(0, -output*0.3),
          // Require the drive
          Robot.driveTrain);

      // reset the settings
      gyro.reset();
        getController().enableContinuousInput(-180, 180);
        getController()
          .setTolerance(3, 0.4);
    }

    @Override
    public void initialize(){
      gyro.reset();
    }

    public static double getAngle(){
      // gets and prints Gyro angle
      System.out.println(gyro.getAngle());
      return gyro.getAngle();
    }
  
    @Override
    public boolean isFinished() {
      // checks if the desired angle is reached within the tolerance
      System.out.println(getController().getSetpoint());
      return getController().atSetpoint();
    }
  }
