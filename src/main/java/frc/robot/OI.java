/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ManualFortuneWheelControl;

import frc.robot.commands.RotationControl;
import frc.robot.commands.TurnToAngleVision;
import frc.robot.commands.DriveToGoal;
import frc.robot.commands.Align;
import frc.robot.commands.Shoot;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public Joystick stick;

  public double xAxis, yAxis, zAxis, twistAxis, throttleAxis;

  static JoystickButton rotationControlButton, manualWHeelControlButton, rotationAlignButton, distanceAlignButton;
  static JoystickButton shootButton;

  public OI() {
    stick = new Joystick(0);

    rotationControlButton = new JoystickButton(stick, 1);
    rotationAlignButton = new JoystickButton(stick, 2);
    distanceAlignButton = new JoystickButton(stick, 3);
    manualWHeelControlButton = new JoystickButton(stick, 4);

    rotationControlButton.whenActive(new RotationControl());
    manualWHeelControlButton.whileHeld(new ManualFortuneWheelControl());
    rotationAlignButton.whenActive(new TurnToAngleVision());

    // Backup
    // distanceAlignButton.whenActive(new DriveToGoal());

    // Pav's
    distanceAlignButton.whenActive(new Align());
    
    shootButton = new JoystickButton(stick, 8);

    shootButton.whenActive(new Shoot());

    //TODO: Align

    //TODO: Climber  
  }

  public void update(){
    // left joystick - x and y
    // right joystick - throttle(y), twist(x) = z

    xAxis = stick.getX();
    yAxis = stick.getY();
    zAxis = stick.getZ();
    twistAxis = stick.getTwist();
    throttleAxis = stick.getThrottle();

    if(Math.abs(twistAxis) < 0.05){
      twistAxis = 0;
    }

    if(Math.abs(yAxis) < 0.05){
      yAxis = 0;
    }
  }
}
