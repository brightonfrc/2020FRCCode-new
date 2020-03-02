package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.DriveForTime;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class QuickAlign extends SequentialCommandGroup {
    /**
     * Creates a new ComplexAuto.
     *
     * @param drive The drive subsystem this command will run on
     * @param hatch The hatch subsystem this command will run on
     */
    public QuickAlign() {
        addCommands(
            // Drive fozrward the specified distance
            new TurnToAngle()

            // new DriveForTime(-0.1, 500)
            
        );
    }   

}
