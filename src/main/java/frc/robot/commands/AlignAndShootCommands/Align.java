/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AlignAndShootCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;

public class Align extends CommandBase {
  /**
   * Creates a new Align (Uses drivetrain only)
   * https://docs.wpilib.org/en/latest/docs/software/vision-processing/grip/reading-array-values-published-by-networktables.html
   */
  DriveTrain driveTrain = new DriveTrain();
  
  // Constructor:
  public Align() {
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  
  @Override
  public void initialize() {
    System.out.println("AnS initialised; HANDS OFF JOYSTICKS");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int x_centre = 69;
    int y_centre = 420; // TODO: GET THESE FROM NETWORK TABLE (X AND Y COORDS OF CENTRE (IMAGE DIMENSIONS 640x480)
    
    double pitch_angle = (y_centre - 239.5)*Constants.PIXEL_DEGREE_VERTICAL_CONVERT;
    double yaw_angle = (x_centre - 319.5)*Constants.PIXEL_DEGREE_HORIZONTAL_CONVERT;
    
    double distance_d = Math.sin(Math.toRadians(90 - Constants.ANGLE_I)) * (Constants.DELTA_H / Math.sin(Math.toRadians(Constants.ANGLE_I + pitch_angle))); //This is the distance from the CAMERA to the centre of target
    double distance_d_adjusted = Math.sqrt((Math.pow(distance_d, 2)) + (Math.pow(Constants.LENGTH_E_C, 2)) - ((2*distance_d*Constants.LENGTH_E_C)*Math.cos(Math.toRadians(180-yaw_angle)))); // This is the distance from the CENTRE OF ROBOT (E) to centre of target
    double yaw_angle_adjusted = distance_d * (distance_d_adjusted / (180 - yaw_angle));
    double left_right_velocity_ratio = (2*(Constants.DISTANCE_D - distance_d_adjusted) - (Constants.DRIVE_WHEEL_TRACK_WIDTH*Math.sin(Math.toRadians(yaw_angle_adjusted))))/(2*(Constants.DISTANCE_D - distance_d_adjusted) + (Constants.DRIVE_WHEEL_TRACK_WIDTH*Math.sin(Math.toRadians(yaw_angle_adjusted)))); //MAINTAIN THIS RATIO BETWEEN THE VELOCITIES OF LEFT AND RIGHT SIDES OF THE TANK DRIVE WHEELS, for time below:
    double proportion_of_total_velocity_right = 0;
    double proportion_of_total_velocity_left = 0;

    //To check if turning correctly
    if(yaw_angle_adjusted < 0) {
      System.out.println("Rotating Anticlockwise");
      if(left_right_velocity_ratio < 1) {
        System.out.println("Ratio Correct");
      } else {
        System.out.println("Ratio INCORRECT");
      }
    } else if(yaw_angle_adjusted > 0) {
      System.out.println("Rotating Clockwise");
      if(left_right_velocity_ratio > 1) {
        System.out.println("Ratio Correct");
      } else {
        System.out.println("Ratio INCORRECT");
      }
    } else {
      System.out.println("NOT Rotating");
    }

    // To find % motor usage of each side of tank drive(determine speeds from this later after tesing drivetrain)
    // In any case, at least one side of the tank drive will be using full power
    if(distance_d > Constants.DISTANCE_D) {
      if(left_right_velocity_ratio < 1) {
        proportion_of_total_velocity_right = 1;
        proportion_of_total_velocity_left = left_right_velocity_ratio;
      } else if(left_right_velocity_ratio > 1) {
        proportion_of_total_velocity_left = 1;
        proportion_of_total_velocity_right = Math.pow(left_right_velocity_ratio, -1);
      } else {
        proportion_of_total_velocity_left = 1;
        proportion_of_total_velocity_right = 1;
      }
    } else if(distance_d < Constants.DISTANCE_D) {
      if(left_right_velocity_ratio < 1) {
        proportion_of_total_velocity_right = -1;
        proportion_of_total_velocity_left = -(left_right_velocity_ratio);
      } else if(left_right_velocity_ratio > 1) {
        proportion_of_total_velocity_left = -1;
        proportion_of_total_velocity_right = -(Math.pow(left_right_velocity_ratio, -1));
      } else {
        proportion_of_total_velocity_left = -1;
        proportion_of_total_velocity_right = -1;
      }
    } else {
      if(yaw_angle_adjusted < 0) {
        proportion_of_total_velocity_right = -1;
        proportion_of_total_velocity_left = 1;
      } else if(yaw_angle_adjusted > 0) {
        proportion_of_total_velocity_right = 1;
        proportion_of_total_velocity_left = -1;
      } else {
        System.out.println("No need to rotate or move");
        end(false);
      }

    // Finding time (seconds) for which to rotate (assuming instant acceleration)
    double rotation_time = (Math.toRadians(yaw_angle_adjusted) * (Constants.DRIVE_WHEEL_TRACK_WIDTH / 2)) / (Math.abs((proportion_of_total_velocity_right * Constants.MAXIMUM_DRIVE_VELOCITY) - (proportion_of_total_velocity_left * Constants.MAXIMUM_DRIVE_VELOCITY)));
    
    //Actally rotating using tank drive.
    driveTrain.tankDrive(proportion_of_total_velocity_left, proportion_of_total_velocity_right);
    // Waiting for the robot to turn
    // try {
    //   long rotation_time_milliseconds = (long) Math.round(rotation_time * 1000); // multiplied by a thousand to convert time to milliseconds
    //   long rotation_cutoff_time_milliseconds = (long) (rotation_time_milliseconds *  Constants.TIME_CUTOFF_PROPORTION);
    //   Thread.sleep(rotation_time_milliseconds - rotation_cutoff_time_milliseconds); // The last constant is just to increase the speed of the entire AnS process by reducing overshoot for each 
    // } catch(InterruptedException ex) {
    //   Thread.currentThread().interrupt();
    // }
    // //Stopping robot (the simple way, no need for PID as command will repeat until threshold yaw angle and d is achieved):
    // driveTrain.tankDrive(0, 0);
    // try {
    //     Thread.sleep(Constants.TIME_DELAY_BEFORE_RESCAN);
    //   } catch(InterruptedException ex) {
    //   Thread.currentThread().interrupt();
    // }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted == false) {
      System.out.println("Successfully Aligned!");
    } else {
      System.out.println("Aligning interrupted, please dont use joysticks when AlignAndShoot is executing.");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int x_centre = 69;
    int y_centre = 420; // TODO: GET THESE FROM NETWORK TABLE (X AND Y COORDS OF CENTRE (IMAGE DIMENSIONS 640x480)

    if((Math.abs(x_centre - 319.5) < Constants.X_TOLERANCE) && (Math.abs(y_centre - 239.5) < Constants.Y_TOLERANCE)){
      return true;
    }
    return false;
  }
}
