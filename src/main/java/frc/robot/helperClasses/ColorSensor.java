/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helperClasses;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.Constants;

/**
 * Colour sensor class with helper functions
 */
public class ColorSensor{
    private ColorSensorV3 m_colorSensor;
    private ColorMatch m_colorMatcher;

    public ColorSensor(Port i2cPort){
        m_colorSensor = new ColorSensorV3(i2cPort);
        m_colorMatcher = new ColorMatch();

        // add these colours to the sensor
        m_colorMatcher.addColorMatch(Constants.BLUE_TARGET);
        m_colorMatcher.addColorMatch(Constants.GREEN_TARGET);
        m_colorMatcher.addColorMatch(Constants.RED_TARGET);
        m_colorMatcher.addColorMatch(Constants.YELLOW_TARGET);
    }

    public ColorMatchResult getColourLabel(){
        Color detectedColor = m_colorSensor.getColor();

        return m_colorMatcher.matchClosestColor(detectedColor);
    }

    // debugs which colour is seen
    public static void debugColorMatch(ColorMatchResult result){
        // System.out.println("Red: " + result.color.red);
        // System.out.println("Green: " + result.color.green);
        // System.out.println("Blue: " + result.color.blue);

        // System.out.println("Confidence: " + result.confidence);

        String colorString = "";

        if (result.color == Constants.BLUE_TARGET) {
          colorString = "Blue";
        } else if (result.color == Constants.RED_TARGET) {
          colorString = "Red";
        } else if (result.color == Constants.GREEN_TARGET) {
          colorString = "Green";
        } else if (result.color == Constants.YELLOW_TARGET) {
          colorString = "Yellow";
        } else {
          colorString = "Unknown";
        }

          System.out.println("Prediction: " + colorString);
    }

    // determines if the colour was recognised
    public static boolean isAColour(ColorMatchResult result){
      if (result.color == Constants.BLUE_TARGET || result.color == Constants.RED_TARGET || result.color == Constants.GREEN_TARGET || result.color == Constants.YELLOW_TARGET) {
        return true;
      } else {
        return false;
      }
    }
}
