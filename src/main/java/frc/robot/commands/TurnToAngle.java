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

// public class TurnToAngle extends PIDCommand {
//   private double m_angle;
//   private double setpoint;

//   public TurnToAngle(double angle) {
//     super(Constants.TURN_P, Constants.TURN_I, RobotMap.TURN_D);
//     super.requires(Robot.driveTrain);
//     this.m_angle = angle;
//     System.out.println(this.m_angle);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {
//     this.setpoint = Robot.sensors.getRotation() + m_angle;

//     SmartDashboard.putNumber("Turn setpoint", this.setpoint);
//     this.pidController.reset();
//     this.setInputRange(-360, 360);
//     this.pidController.setOutputRange(-RobotMap.MAX_TURN_SPEED, RobotMap.MAX_TURN_SPEED);
//     this.pidController.setAbsoluteTolerance(RobotMap.GYRO_GAY);
//     super.setSetpoint(this.setpoint);

//     Robot.driveTrain.setLeftMotor(0);
//     Robot.driveTrain.setRightMotor(0);

//     super.setTimeout(RobotMap.COMMAND_TIMEOUT);
//     this.pidController.enable();
//   }

//   @Override
//   protected void execute(){
//     SmartDashboard.putNumber("PID OUT", this.pidController.get());
//   }
//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   protected boolean isFinished() {
//     return super.isTimedOut() || this.pidController.onTarget();
//   }

//   // Called once after isFinished returns true
//   @Override
//   protected void end() {
//     this.pidController.disable();
//   }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   protected void interrupted() {
//     this.end();
//   }

//   @Override
//   protected double returnPIDInput() {
//     return Robot.sensors.getRotation();
//   }

//   @Override
//   protected void usePIDOutput(double output) {
//     Robot.driveTrain.setRightMotor(-output);
//     Robot.driveTrain.setLeftMotor(output);
//   }
// }

public class TurnToAngle extends PIDCommand {
    /**
     * Turns to robot to the specified angle.
     *
     * @param targetAngleDegrees The angle to turn to
     * @param drive              The drive subsystem to use
     */

    private static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    public TurnToAngle(double targetAngleDegree) {
      super(
          new PIDController(Constants.DRIVETRAIN_ROTATION_P, Constants.DRIVETRAIN_ROTATION_I, Constants.DRIVETRAIN_ROTATION_D),
          // Close loop on heading
          TurnToAngle::getAngle,
          // Set reference to target
          targetAngleDegree,
          // Pipe output to turn robot
          output -> Robot.driveTrain.arcadeDrive(0, -output*0.3),
          // Require the drive
          Robot.driveTrain);

      gyro.reset();
  
      // Set the controller to be continuous (because it is an angle controller)
      getController().enableContinuousInput(-180, 180);
      // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
      // setpoint before it is considered as having reached the reference
      getController()
          .setTolerance(3, 0.9);
    }

    @Override
    public void initialize(){
      gyro.reset();
    }

    public static double getAngle(){
      return gyro.getAngle();
    }
  
    @Override
    public boolean isFinished() {
      System.out.println(getAngle() - 30);
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
  }
  