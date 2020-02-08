// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Shooter extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonSRX shooterDrive1;
private WPI_TalonSRX shooterDrive2;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private final static double SPEED_P_CONSTANT_TOP = 5.0;
    private final static double SPEED_I_CONSTANT_TOP = 0.0025;
    private final static double SPEED_D_CONSTANT_TOP = 0.0;
    private final static double SPEED_F_CONSTANT_TOP = 0.0;

    private final static double SPEED_P_CONSTANT_BOTTOM = 5.0;
    private final static double SPEED_I_CONSTANT_BOTTOM = 0.0025;
    private final static double SPEED_D_CONSTANT_BOTTOM = 0.0;
    private final static double SPEED_F_CONSTANT_BOTTOM = 0.0;

    public double speedP_top = SPEED_P_CONSTANT_TOP;
    public double speedI_top = SPEED_I_CONSTANT_TOP;
    public double speedD_top = SPEED_D_CONSTANT_TOP;
    public double speedF_top = SPEED_F_CONSTANT_TOP;

    public double speedP_bottom = SPEED_P_CONSTANT_BOTTOM;
    public double speedI_bottom = SPEED_I_CONSTANT_BOTTOM;
    public double speedD_bottom = SPEED_D_CONSTANT_BOTTOM;
    public double speedF_bottom = SPEED_F_CONSTANT_BOTTOM;


    public final static int PID_SLOT_SPEED_MODE = 0;
    private final int TIMEOUT_MS = 10;

    private static final int MAX_TICKS_PER_SEC_TOP = 280;
    private static final int MAX_TICKS_PER_SEC_BOTTOM = 280;

    

    public Shooter() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
shooterDrive1 = new WPI_TalonSRX(8);


        
shooterDrive2 = new WPI_TalonSRX(9);


        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void shoot(double rotation){
        this.setPercentSpeedPIDTop(rotation);
        this.setPercentSpeedPIDBottom(rotation);
    }
    public void initSpeedMode() {    	
    	shooterDrive1.set(ControlMode.Velocity, 0);
        shooterDrive1.config_kP(PID_SLOT_SPEED_MODE, speedP_top, TIMEOUT_MS);
    	shooterDrive1.config_kI(PID_SLOT_SPEED_MODE, speedI_top, TIMEOUT_MS);
    	shooterDrive1.config_kD(PID_SLOT_SPEED_MODE, speedD_top, TIMEOUT_MS);
    	shooterDrive1.config_kF(PID_SLOT_SPEED_MODE, speedF_top, TIMEOUT_MS);
        shooterDrive1.selectProfileSlot(PID_SLOT_SPEED_MODE, 0);

        shooterDrive2.set(ControlMode.Velocity, 0);
        shooterDrive2.config_kP(PID_SLOT_SPEED_MODE, speedP_bottom, TIMEOUT_MS);
    	shooterDrive2.config_kI(PID_SLOT_SPEED_MODE, speedI_bottom, TIMEOUT_MS);
    	shooterDrive2.config_kD(PID_SLOT_SPEED_MODE, speedD_bottom, TIMEOUT_MS);
    	shooterDrive2.config_kF(PID_SLOT_SPEED_MODE, speedF_bottom, TIMEOUT_MS);
        shooterDrive2.selectProfileSlot(PID_SLOT_SPEED_MODE, 0);
    }
    public void stop(){
        shooterDrive1.setNeutralMode(NeutralMode.Brake);
        shooterDrive2.setNeutralMode(NeutralMode.Brake);
        shoot(0);
    }
    public void setPercentSpeedPIDTop(double setSpeed) {
        shooterDrive1.set(ControlMode.Velocity, MAX_TICKS_PER_SEC_TOP * setSpeed);
    }
    public void setPercentSpeedPIDBottom(double setSpeed){
        shooterDrive2.set(ControlMode.Velocity, MAX_TICKS_PER_SEC_BOTTOM * setSpeed);
    }
    public int getTicksPerSecondTop(){
        return shooterDrive1.getSelectedSensorVelocity();
    }
    public int getTicksPerSecondBottom(){
        return shooterDrive2.getSelectedSensorVelocity();
    }
}

