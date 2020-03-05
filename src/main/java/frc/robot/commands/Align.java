/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Align extends PIDCommand {
  /**
   * Creates a new Align.
   */

  private static double yaw_angle_adjusted, distance_d_adjusted;

  private static double turnToAngle;

  private static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public Align() {
    super(
        // The controller that the command will use
        new PIDController(Constants.DRIVETRAIN_ROTATION_P, Constants.DRIVETRAIN_ROTATION_I, Constants.DRIVETRAIN_ROTATION_D),
        // This should return the measurement
        Align::getError,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        Align::correctForError
        );

    gyro.reset();
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(3, 0.4);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    System.out.println("AnS initialised; HANDS OFF JOYSTICKS");
    updateAngleAndDistance();

    gyro.reset();
  }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {

    
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted == false) {
      System.out.println("Successfully Aligned!");
    } else {
      System.out.println("Aligning interrupted, please dont use joysticks when AlignAndShoot is executing.");
    }
  }

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   if (distanceControl.getController().atSetpoint() && angleControl.getController().atSetpoint()) {
  //     return true;
  //   }
  //   return false;
  // }

  private static double getError(){
    return gyro.getAngle() - turnToAngle;
  }

  private static void correctForError(double errorMultiplier){

    double left_right_velocity_ratio = (2 * (Constants.DISTANCE_D - distance_d_adjusted)
        - (Constants.DRIVE_WHEEL_TRACK_WIDTH * Math.sin(Math.toRadians(yaw_angle_adjusted))))
        / (2 * (Constants.DISTANCE_D - distance_d_adjusted)
            + (Constants.DRIVE_WHEEL_TRACK_WIDTH * Math.sin(Math.toRadians(yaw_angle_adjusted)))); // MAINTAIN THIS
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
    double rightRaw = 1;// 1

    double maxPower = Math.max(leftRaw, rightRaw);

    proportion_of_total_velocity_left = leftRaw / maxPower;// 1 or l/r
    proportion_of_total_velocity_right = rightRaw / maxPower;// r/l or 1

    // Actally rotating using tank drive.
    Robot.driveTrain.tankDrive(proportion_of_total_velocity_left * errorMultiplier, proportion_of_total_velocity_right * errorMultiplier);

    // // Finding time (seconds) for which to rotate (assuming instant acceleration)
    // double rotation_time = (Math.toRadians(yaw_angle_adjusted) *
    // (Constants.DRIVE_WHEEL_TRACK_WIDTH / 2)) /
    // (Math.abs((proportion_of_total_velocity_right *
    // Constants.MAXIMUM_DRIVE_VELOCITY) - (proportion_of_total_velocity_left *
    // Constants.MAXIMUM_DRIVE_VELOCITY)));
  }

  private void updateAngleAndDistance(){
    double pitch = Robot.pitchEntry.getDouble(0.0);
    double yaw = Robot.yawEntry.getDouble(0.0);


    // update the values for the PID
    
    // these are the values for the PID
    distance_d_adjusted = Robot.computerVision.getdAdjusted(pitch, yaw);
    yaw_angle_adjusted = Robot.computerVision.getYawAdjusted();

    turnToAngle = yaw_angle_adjusted;
  }


  private static void debugRatio(double yaw_angle_adjusted, double left_right_velocity_ratio) {
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
