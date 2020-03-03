/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;

public class TurnToAngleVision extends PIDCommand {
    /**
     * Turns to robot to the specified angle.
     *
     * @param targetAngleDegrees The angle to turn to
     * @param drive              The drive subsystem to use
     */

    private static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private static double turnToAngle;

    public TurnToAngleVision() {
      super(
          new PIDController(Constants.DRIVETRAIN_ROTATION_P, Constants.DRIVETRAIN_ROTATION_I, Constants.DRIVETRAIN_ROTATION_D),
          // Close loop on heading
          TurnToAngleVision::getAngle,
          // Set reference to target
          0,
          // Pipe output to turn robot
          output -> Robot.driveTrain.arcadeDrive(0, -output*0.3),
          // Require the drive
          Robot.driveTrain);

      gyro.reset();
        getController().enableContinuousInput(-180, 180);
      getController()
          .setTolerance(3, 0.4);
    }

    @Override
    public void initialize(){
      gyro.reset();
      turnToAngle = Robot.yawEntry.getDouble(0.0);
    }

    public static double getAngle(){
      System.out.println(gyro.getAngle()-turnToAngle);
      return gyro.getAngle()-turnToAngle;
    }
  
    @Override
    public boolean isFinished() {
      System.out.println(getController().getSetpoint());
      return getController().atSetpoint();
    }
  }
