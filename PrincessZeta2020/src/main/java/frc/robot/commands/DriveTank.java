// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// it from being updated in the future.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight;
import frc.robot.Robot;

/**
 *
 */
public class DriveTank extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public DriveTank() {

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        Limelight.setLedMode(Limelight.LightMode.iOff);
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveTrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveTrain.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        
        double leftControllerY = Robot.oi.driverLeftJoystick.getY();
        double rightControllerY = Robot.oi.driverRightJoystick.getY();
        double leftSpeed = -leftControllerY;
        double rightSpeed = -rightControllerY;
        
        if (Math.abs(leftSpeed) <= .08 && Math.abs(rightSpeed) >= .08){
            Robot.driveTrain.drive(0, rightSpeed);
        } else if (Math.abs(rightSpeed) <= .08 && Math.abs(leftSpeed) >= .08){
            Robot.driveTrain.drive(leftSpeed, 0);
        } else if (Math.abs(rightSpeed) <= .08 && Math.abs(leftSpeed) <= .08){
            Robot.driveTrain.drive(0, 0);
        } else {
            Robot.driveTrain.drive(leftSpeed, rightSpeed);
        }
        
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
