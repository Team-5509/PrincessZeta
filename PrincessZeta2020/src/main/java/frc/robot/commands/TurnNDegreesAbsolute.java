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
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class TurnNDegreesAbsolute extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double m_degrees;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    private double driveSpeed;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public TurnNDegreesAbsolute(double degrees) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        m_degrees = degrees;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveTrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        //FIXME: As these are pretty much constant for the turn PID controller, can probably move them to DriveTrain's constructor
        Robot.driveTrain.turnEnableContinuousInput(-180, 180);
        Robot.driveTrain.turnSetTolerance(1);
        //FIXME: Parameter validation.
        // This should only take angles -180 to 180.
        // If something outside this range is given, either:
        // a) Fix it (+/-360 until it hits the range)
        // b) Throw an IllegalArgumentException
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.shifter.shiftLow();
        // TODO: Time how long it'll take in low gear to complete a 180 degree turn. That'll be the max.
        setTimeout(2);
        Robot.driveTrain.setTurnSetpoint(m_degrees);    
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        driveSpeed = Robot.driveTrain.turnCalculate(Robot.driveTrain.getGyroTurnAngle());
        //FIXME: Use driveRaw 
        Robot.driveTrain.drive(driveSpeed, driveSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() { 
        return isTimedOut() || Robot.driveTrain.turnAtSetpoint();
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
