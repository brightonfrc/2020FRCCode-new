/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AlignAndShootCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.Constants;

public class Align extends CommandBase {
  /**
   * Creates a new Align (Uses drivetrain only)
   * https://docs.wpilib.org/en/latest/docs/software/vision-processing/grip/reading-array-values-published-by-networktables.html
   */

  // these values are constantly updated
  double yaw_angle_adjusted = 0;
  double distance_d_adjusted = 0;

  double yaw_angle_multiplier = 1;
  double distance_angle_multiplier = 1;

  private int x_centre = 69;
  private int y_centre = 420; // TODO: GET THESE FROM NETWORK TABLE (X AND Y COORDS OF CENTRE (IMAGE
                              // DIMENSIONS 640x480)

  private PIDSubsystem distanceControl, angleControl;

  // Constructor:
  public Align() {
    addRequirements(Robot.driveTrain);

    distanceControl = new PIDSubsystem(new PIDController(Constants.DRIVETRAIN_THRUST_P, Constants.DRIVETRAIN_THRUST_I, Constants.DRIVETRAIN_THRUST_D)){
    
      @Override
      protected void useOutput(double output, double setpoint) {
        // set the values
        distance_angle_multiplier = output;
      }
    
      @Override
      protected double getMeasurement() {
        // the distance adjusted should equal D, so
        return Constants.DISTANCE_D - distance_d_adjusted;
      }
    };

    angleControl = new PIDSubsystem(new PIDController(Constants.DRIVETRAIN_ROTATION_P, Constants.DRIVETRAIN_ROTATION_I, Constants.DRIVETRAIN_ROTATION_D)){
    
      @Override
      protected void useOutput(double output, double setpoint) {
        yaw_angle_multiplier = output;
      }
    
      @Override
      protected double getMeasurement() {
        // the angle should be 0
        return yaw_angle_adjusted;
      }
    };
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    System.out.println("AnS initialised; HANDS OFF JOYSTICKS");

    // TODO: set the tolerances and setPoints

    // reset the controllers
    distanceControl.getController().reset();
    angleControl.getController().reset();

    distanceControl.getController().setTolerance(Constants.DRIVETRAIN_DISTANCE_TOLERANCE);
    angleControl.getController().setTolerance(Constants.DRIVETRAIN_ANGLE_TOLERANCE);

    distanceControl.setSetpoint(0);
    angleControl.setSetpoint(0);

    distanceControl.enable();
    angleControl.enable();

    // TODO: update x and y centre

    yaw_angle_multiplier = 1;
    distance_angle_multiplier = 1;

    // find left to right ratio
    double[] yawAnddAdjusted = Robot.computerVision.getYawAnddAdjusted(x_centre, y_centre);

    // these are the values for the PID
    yaw_angle_adjusted = yawAnddAdjusted[0];
    distance_d_adjusted = yawAnddAdjusted[1];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double left_right_velocity_ratio = (2 * (Constants.DISTANCE_D - distance_d_adjusted * distance_angle_multiplier)
        - (Constants.DRIVE_WHEEL_TRACK_WIDTH * Math.sin(Math.toRadians(yaw_angle_adjusted * yaw_angle_multiplier))))
        / (2 * (Constants.DISTANCE_D - distance_d_adjusted * distance_angle_multiplier)
            + (Constants.DRIVE_WHEEL_TRACK_WIDTH * Math.sin(Math.toRadians(yaw_angle_adjusted * yaw_angle_multiplier)))); // MAINTAIN THIS
                                                                                                   // RATIO BETWEEN THE
                                                                                                   // VELOCITIES OF LEFT
                                                                                                   // AND RIGHT SIDES OF
                                                                                                   // THE TANK DRIVE
                                                                                                   // WHEELS, for time
                                                                                                   // below:
    double proportion_of_total_velocity_right = 0;
    double proportion_of_total_velocity_left = 0;

    debugRatio(yaw_angle_adjusted, left_right_velocity_ratio);

    // To check if turning correctly

    // To find % motor usage of each side of tank drive(determine speeds from this
    // later after tesing drivetrain)
    // In any case, at least one side of the tank drive will be using full power

    double leftRaw = left_right_velocity_ratio;// l/r
    double rightRaw = 1;// l

    double maxPower = Math.max(leftRaw, rightRaw);

    proportion_of_total_velocity_left = leftRaw / maxPower;// 1 or l/r
    proportion_of_total_velocity_right = rightRaw / maxPower;// r/l or 1

    // Actally rotating using tank drive.
    Robot.driveTrain.tankDrive(proportion_of_total_velocity_left, proportion_of_total_velocity_right);

    // if(distance_d > Constants.DISTANCE_D) {
    // if(left_right_velocity_ratio < 1) {
    // proportion_of_total_velocity_right = 1;
    // proportion_of_total_velocity_left = left_right_velocity_ratio;
    // } else if(left_right_velocity_ratio > 1) {
    // proportion_of_total_velocity_left = 1;
    // proportion_of_total_velocity_right = Math.pow(left_right_velocity_ratio, -1);
    // } else {
    // proportion_of_total_velocity_left = 1;
    // proportion_of_total_velocity_right = 1;
    // }
    // } else if(distance_d < Constants.DISTANCE_D) {
    // if(left_right_velocity_ratio < 1) {
    // proportion_of_total_velocity_right = -1;
    // proportion_of_total_velocity_left = -(left_right_velocity_ratio);
    // } else if(left_right_velocity_ratio > 1) {
    // proportion_of_total_velocity_left = -1;
    // proportion_of_total_velocity_right = -(Math.pow(left_right_velocity_ratio,
    // -1));
    // } else {
    // proportion_of_total_velocity_left = -1;
    // proportion_of_total_velocity_right = -1;
    // }
    // } else {
    // if(yaw_angle_adjusted < 0) {
    // proportion_of_total_velocity_right = -1;
    // proportion_of_total_velocity_left = 1;
    // } else if(yaw_angle_adjusted > 0) {
    // proportion_of_total_velocity_right = 1;
    // proportion_of_total_velocity_left = -1;
    // } else {
    // System.out.println("No need to rotate or move");
    // }

    // // Finding time (seconds) for which to rotate (assuming instant acceleration)
    // double rotation_time = (Math.toRadians(yaw_angle_adjusted) *
    // (Constants.DRIVE_WHEEL_TRACK_WIDTH / 2)) /
    // (Math.abs((proportion_of_total_velocity_right *
    // Constants.MAXIMUM_DRIVE_VELOCITY) - (proportion_of_total_velocity_left *
    // Constants.MAXIMUM_DRIVE_VELOCITY)));

    // Waiting for the robot to turn
    // try {
    // long rotation_time_milliseconds = (long) Math.round(rotation_time * 1000); //
    // multiplied by a thousand to convert time to milliseconds
    // long rotation_cutoff_time_milliseconds = (long) (rotation_time_milliseconds *
    // Constants.TIME_CUTOFF_PROPORTION);
    // Thread.sleep(rotation_time_milliseconds - rotation_cutoff_time_milliseconds);
    // // The last constant is just to increase the speed of the entire AnS process
    // by reducing overshoot for each
    // } catch(InterruptedException ex) {
    // Thread.currentThread().interrupt();
    // }
    // //Stopping robot (the simple way, no need for PID as command will repeat
    // until threshold yaw angle and d is achieved):
    // driveTrain.tankDrive(0, 0);
    // try {
    // Thread.sleep(Constants.TIME_DELAY_BEFORE_RESCAN);
    // } catch(InterruptedException ex) {
    // Thread.currentThread().interrupt();
    // }

    // TODO: update x and y centre

    // update the values for the PID
    double[] yawAnddAdjusted = Robot.computerVision.getYawAnddAdjusted(x_centre, y_centre);

    // these are the values for the PID
    yaw_angle_adjusted = yawAnddAdjusted[0];
    distance_d_adjusted = yawAnddAdjusted[1];
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    distanceControl.disable();
    angleControl.disable();

    if (interrupted == false) {
      System.out.println("Successfully Aligned!");
    } else {
      System.out.println("Aligning interrupted, please dont use joysticks when AlignAndShoot is executing.");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (distanceControl.getController().atSetpoint() && angleControl.getController().atSetpoint()) {
      return true;
    }
    return false;
  }

  private void debugRatio(double yaw_angle_adjusted, double left_right_velocity_ratio) {
    if (yaw_angle_adjusted < 0) {
      System.out.println("Rotating Anticlockwise");
      if (left_right_velocity_ratio < 1) {
        System.out.println("Ratio Correct");
      } else {
        System.out.println("Ratio INCORRECT");
      }
    } else if (yaw_angle_adjusted > 0) {
      System.out.println("Rotating Clockwise");
      if (left_right_velocity_ratio > 1) {
        System.out.println("Ratio Correct");
      } else {
        System.out.println("Ratio INCORRECT");
      }
    } else {
      System.out.println("NOT Rotating");
    }
  }
}
