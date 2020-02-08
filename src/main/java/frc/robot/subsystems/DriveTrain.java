/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.customDatatypes.DriveSignal;
import frc.robot.customDatatypes.Twist2d;

/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {
  private Talon m_motorLeft1;
  private Talon m_motorLeft2;
  private Talon m_motorRight1;
  private Talon m_motorRight2;

  // create groups for motors
  private SpeedControllerGroup m_leftMotors;
  private SpeedControllerGroup m_rightMotors;

  private DifferentialDrive m_differentialDrive;


  public DriveTrain() {
    m_motorLeft1 = new Talon(Constants.MOTOR_LEFT_1_ID);
    m_motorLeft2 = new Talon(Constants.MOTOR_LEFT_2_ID);
    m_motorRight1 = new Talon(Constants.MOTOR_RIGHT_1_ID);
    m_motorRight2 = new Talon(Constants.MOTOR_RIGHT_2_ID);

    // WAY 1 (on one note)
    m_leftMotors = new SpeedControllerGroup(m_motorLeft1, m_motorLeft2);
    m_rightMotors = new SpeedControllerGroup(m_motorRight1, m_motorRight2);

    m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  }

  // TODO: check if user input is less than epsilon in OI

  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  // drives with set speeds for each side of motors
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void tankDrive(DriveSignal driveSignal) {
    m_differentialDrive.tankDrive(driveSignal.getLeftPercentage(), driveSignal.getRightPercentage());
  }

  // drive with a forward speed and rotation speed
  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    m_differentialDrive.arcadeDrive(moveSpeed, rotateSpeed);
  }

  // TODO: do not forget to change the OI input
  public void curvatureDrive(double throttle, double curvatureInput, double inverseKinematicsTurnThreshold) {
    // get the required amount of motor powers to turn
    DriveSignal signal = DriveSignal.inverseKinematics(new Twist2d(throttle, 0.0, curvatureInput),
        inverseKinematicsTurnThreshold);
    // make sure that no motors go above 100% speed
    double scalingFactor = Math.max(1.0,
        Math.max(Math.abs(signal.getLeftPercentage()), Math.abs(signal.getRightPercentage())));
    // apply the scale factor
    tankDrive(signal.getLeftPercentage() / scalingFactor, signal.getRightPercentage() / scalingFactor);
  }

  // if no threshold
  public void curvatureDrive(double throttle, double curvatureInput) {
    curvatureDrive(throttle, curvatureInput, 0.0);
  }

  public void quickTurn(double rotationSpeed) {
    tankDrive(rotationSpeed, -rotationSpeed);
  }
}
