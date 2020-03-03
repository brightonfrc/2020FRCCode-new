/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriverControls;
import frc.robot.customDatatypes.DriveSignal;
import frc.robot.customDatatypes.Twist2d;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {
  private final VictorSPX m_motorLeft1;
  private final VictorSPX m_motorLeft2;
  private final VictorSPX m_motorRight1;
  private final VictorSPX m_motorRight2;

  // create groups for motors
  // private final SpeedControllerGroup m_leftMotors;
  // private final SpeedControllerGroup m_rightMotors;

  // private final DifferentialDrive m_differentialDrive;

  public DriveTrain() {
    m_motorLeft1 = new VictorSPX(Constants.MOTOR_LEFT_1_ID);
    m_motorLeft2 = new VictorSPX(Constants.MOTOR_LEFT_2_ID);
    m_motorRight1 = new VictorSPX(Constants.MOTOR_RIGHT_1_ID);
    m_motorRight2 = new VictorSPX(Constants.MOTOR_RIGHT_2_ID);

    m_motorLeft2.follow(m_motorLeft1);
    m_motorRight2.follow(m_motorRight1);

    // // WAY 1 (on one note)
    // m_leftMotors = new SpeedControllerGroup(m_motorLeft1, m_motorLeft2);
    // m_rightMotors = new SpeedControllerGroup(m_motorRight1, m_motorRight2);

    // m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  }

  public void initDefaultCommand() {
    super.setDefaultCommand(new DriverControls());
  }

  public void setLeftMotors(double speed){
    this.m_motorLeft1.set(ControlMode.PercentOutput, speed);
  }

  public void setRightMotors(double speed){
    this.m_motorRight1.set(ControlMode.PercentOutput, speed);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // drives with set speeds for each side of motor
  // Input speeds from 0 to 1 (1 is full speed)
  public void tankDrive(final double leftSpeed, final double rightSpeed) {
    setLeftMotors(-leftSpeed);
    setRightMotors(rightSpeed);
  }

  public void tankDrive(final DriveSignal driveSignal) {
    setLeftMotors(driveSignal.getLeftPercentage());
    setRightMotors(driveSignal.getRightPercentage());
  }

  // drive with a forward speed and rotation speed
  public void arcadeDrive(final double moveSpeed, final double rotateSpeed) {
    double leftSpeed = moveSpeed + rotateSpeed;
    double rightSpeed = moveSpeed - rotateSpeed;

    // make sure that the motors do not go above 100%
    double scalingFactor = Math.max(1.0, Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed)));

    leftSpeed /= scalingFactor;
    rightSpeed /= scalingFactor;

    tankDrive(leftSpeed, rightSpeed);
  }

  public void curvatureDrive(double throttle, double curvatureInput,
      boolean isManual) {
    
    double inverseKinematicsTurnThreshold = isManual? Constants.MANUAL_TURN_THRESHOLD: 0d;
    double quickTurnThrustThreshold = isManual? Constants.MANUAL_QUICK_TURN_THROTTLE_THRESHOLD: 0d;



    // if quick turn
    if(Math.abs(throttle) < quickTurnThrustThreshold){
      // set throttle to 0
      throttle = 0d;
    }

    // get the required amount of motor powers to turn
    final DriveSignal signal = DriveSignal.inverseKinematics(new Twist2d(throttle, 0.0, curvatureInput),
        inverseKinematicsTurnThreshold);
    // make sure that no motors go above 100% speed
    final double scalingFactor = Math.max(1.0,
        Math.max(Math.abs(signal.getLeftPercentage()), Math.abs(signal.getRightPercentage())));

    // apply the scale factor
    tankDrive(signal.getLeftPercentage() / scalingFactor, signal.getRightPercentage() / scalingFactor);
  }

  // if no threshold
  public void curvatureDrive(final double throttle, final double curvatureInput) {
    curvatureDrive(throttle, curvatureInput, false);
  }
}