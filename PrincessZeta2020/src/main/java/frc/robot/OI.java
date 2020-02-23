// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import frc.robot.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
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

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public JoystickButton shiftHighButton;
public JoystickButton intakeInButton;
public JoystickButton alignToTargetButton;
public JoystickButton toggleFoldButton;
public Joystick driverRightJoystick;
public JoystickButton shiftLowButton;
public JoystickButton intakeOutButton;
public Joystick driverLeftJoystick;
public JoystickButton rotationControlButton;
public JoystickButton conveyUpButton;
public JoystickButton conveyDownButton;
public JoystickButton autoShootButton;
public JoystickButton winchUpButton;
public JoystickButton winchDownButton;
public Joystick copilotControl;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

copilotControl = new Joystick(2);

winchDownButton = new JoystickButton(copilotControl, 3);
winchDownButton.whileHeld(new Winching(-1));
winchUpButton = new JoystickButton(copilotControl, 4);
winchUpButton.whileHeld(new Winching(1));
autoShootButton = new JoystickButton(copilotControl, 2);
autoShootButton.whileHeld(new ShootingAuto());
conveyDownButton = new JoystickButton(copilotControl, 6);
conveyDownButton.whileHeld(new Conveying(-1));
conveyUpButton = new JoystickButton(copilotControl, 5);
conveyUpButton.whileHeld(new Conveying(1));
rotationControlButton = new JoystickButton(copilotControl, 1);
rotationControlButton.whenPressed(new RotationControl());
driverLeftJoystick = new Joystick(1);

intakeOutButton = new JoystickButton(driverLeftJoystick, 1);
intakeOutButton.whileHeld(new Intaking(-.50));
shiftLowButton = new JoystickButton(driverLeftJoystick, 4);
shiftLowButton.whenPressed(new ShiftLow());
driverRightJoystick = new Joystick(0);

toggleFoldButton = new JoystickButton(driverRightJoystick, 2);
toggleFoldButton.whenPressed(new ToggleFold());
alignToTargetButton = new JoystickButton(driverRightJoystick, 12);
alignToTargetButton.whenPressed(new AlignDrivetrainToTarget());
intakeInButton = new JoystickButton(driverRightJoystick, 1);
intakeInButton.whileHeld(new Intaking(.50));
shiftHighButton = new JoystickButton(driverRightJoystick, 3);
shiftHighButton.whenPressed(new ShiftHigh());


        // SmartDashboard Buttons
        SmartDashboard.putData("DriveTank", new DriveTank());
        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("DriveClayton", new DriveClayton());
        SmartDashboard.putData("RotationControl", new RotationControl());
        SmartDashboard.putData("PositionControl", new PositionControl());
        SmartDashboard.putData("Shooting", new Shooting());
        SmartDashboard.putData("ShiftHigh", new ShiftHigh());
        SmartDashboard.putData("ShiftLow", new ShiftLow());
        SmartDashboard.putData("FoldOut", new FoldOut());
        SmartDashboard.putData("FoldIn", new FoldIn());
        SmartDashboard.putData("AlignDrivetrainToTarget", new AlignDrivetrainToTarget());
        SmartDashboard.putData("ShootingAuto", new ShootingAuto());
        SmartDashboard.putData("TurnNDegreesAbsolute: zeroDegrees", new TurnNDegreesAbsolute(0));
        SmartDashboard.putData("TurnNDegreesAbsolute: right90Degrees", new TurnNDegreesAbsolute(90));
        SmartDashboard.putData("TurnNDegreesAbsolute: one80Degrees", new TurnNDegreesAbsolute(180));
        SmartDashboard.putData("TurnNDegreesAbsolute: left90Degrees", new TurnNDegreesAbsolute(-90));
        SmartDashboard.putData("DriveXFeet: DriveTrain_Drive_10_feet", new DriveXFeet(10, .4));
        SmartDashboard.putData("GoalSideOffsetRight", new GoalSideOffsetRight());
        SmartDashboard.putData("GoalSideCentered", new GoalSideCentered());
        SmartDashboard.putData("GoalSideOffsetLeft", new GoalSideOffsetLeft());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public Joystick getdriverRightJoystick() {
        return driverRightJoystick;
    }

public Joystick getdriverLeftJoystick() {
        return driverLeftJoystick;
    }

public Joystick getcopilotControl() {
        return copilotControl;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

