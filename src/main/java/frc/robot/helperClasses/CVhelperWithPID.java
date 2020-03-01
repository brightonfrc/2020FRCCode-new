/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helperClasses;

/**
 * Add your docs here.
 */
public class CVhelperWithPID {
    private double variableSinceLastKnownPosition = 0;
    private double deltaAfterLastFrameTaken = 0;

    public CVhelperWithPID(){
        restart();
    }

    public void restart(){
        variableSinceLastKnownPosition = 0;
        deltaAfterLastFrameTaken = 0;
    }

    public void newFrame(double newVar){
        variableSinceLastKnownPosition = newVar + deltaAfterLastFrameTaken;
        deltaAfterLastFrameTaken = 0;
    }

    public void updateFromSensor(double change){
        deltaAfterLastFrameTaken += change;
    }

    public void setDeltaAfterLastFrameTaken(double newDelta){
        deltaAfterLastFrameTaken = newDelta;
    }

    public double getPredictedValue(){
        return variableSinceLastKnownPosition + deltaAfterLastFrameTaken;
    }
}
