// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;

/**
 *
 */
public class PositionControl extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public PositionControl() {

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.controlPanel);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        String gameData;
        Color color = Robot.controlPanel.getColor();
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
            case 'B':
                if (Robot.controlPanel.isRed(color)) {
                    Robot.controlPanel.stopRotation();
                } else {
                    Robot.controlPanel.rotateControlPanel(.5);
                }
                break;
            case 'G':
                if (Robot.controlPanel.isYellow(color)) {
                    Robot.controlPanel.stopRotation();
                } else {
                    Robot.controlPanel.rotateControlPanel(.5);
                }
                break;
            case 'R':
                if (Robot.controlPanel.isBlue(color)) {
                    Robot.controlPanel.stopRotation();
                } else {
                    Robot.controlPanel.rotateControlPanel(.5);
                }
                break;
            case 'Y':
                if (Robot.controlPanel.isGreen(color)) {
                    Robot.controlPanel.stopRotation();
                } else {
                    Robot.controlPanel.rotateControlPanel(.5);
                }
                break;
            default:
                // This is corrupt data
                break;
            }
        } else {

        }

        // TODO: create code for position control
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.controlPanel.stopRotation();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
