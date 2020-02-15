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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI.Port;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class DriveTrain extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private static CANSparkMax frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private static CANEncoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private static CANPIDController CANPidController;
    // private WPI_TalonSRX frontLeftDrive, frontRightDrive, backLeftDrive,
    // backRightDrive;

    // PID Constants (all values still need to be changed, these are values for
    // plybot)
    private static final double KP_DRIVE = 0.25;
    private static final double KI_DRIVE = 0.0001; // lowered
    private static final double KD_DRIVE = 0.0;
    private static final double KIZ_DRIVE = 0;
    private static final double KFF_DRIVE = 0.000015;
    // private static final double KFF = 0.12;

    private static final int TIMEOUT_MS = 10;
    private static final int K_MAX_OUTPUT = 1;
    private static final int K_MIN_OUTPUT = -1;
    private static final int MAX_TICKS_PER_SECOND = 9000;
    private static final int TICKS_PER_FOOT = 5270;
    private static final int MAX_RPM = 5874;

    // These need to be tuned for turn control
    private static double PIDTURN_P = 0.05;
    private static double PIDTURN_I = 0.00004;
    private static double PIDTURN_D = 0.0025;

    public PIDController pidController;

    private static final AHRS ahrs = new AHRS(Port.kMXP); // NAVx

    public DriveTrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        frontLeftDrive = new CANSparkMax(1, MotorType.kBrushless);
        frontRightDrive = new CANSparkMax(2, MotorType.kBrushless);
        backLeftDrive = new CANSparkMax(3, MotorType.kBrushless);
        backRightDrive = new CANSparkMax(4, MotorType.kBrushless);

        /*
         * frontLeftDrive = new WPI_TalonSRX(1); frontRightDrive = new WPI_TalonSRX(2);
         * backLeftDrive = new WPI_TalonSRX(3); backRightDrive = new WPI_TalonSRX(4);
         */

        backLeftDrive.follow(frontLeftDrive);
        backRightDrive.follow(frontRightDrive);

        frontLeftEncoder = new CANEncoder(frontLeftDrive);
        frontRightEncoder = new CANEncoder(frontRightDrive);
        backLeftEncoder = new CANEncoder(backLeftDrive);
        frontRightEncoder = new CANEncoder(backRightDrive);

        pidController = new PIDController(PIDTURN_P, PIDTURN_I, PIDTURN_D);
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("DriveXFeet P Gain", KP_DRIVE);
        SmartDashboard.putNumber("DriveXFeet I Gain", KI_DRIVE);
        SmartDashboard.putNumber("DriveXFeet D Gain", KD_DRIVE);
        SmartDashboard.putNumber("DriveXFeet I Zone", KIZ_DRIVE);
        SmartDashboard.putNumber("DriveXFeet Feed Forward", KFF_DRIVE);
        SmartDashboard.putNumber("DriveXFeet Max Output", K_MAX_OUTPUT);
        SmartDashboard.putNumber("DriveXFeet Min Output", K_MIN_OUTPUT);

        // set PID coefficients
        /*CANPidController.setP(KP_DRIVE);
        CANPidController.setI(KI_DRIVE);
        CANPidController.setD(KD_DRIVE);
        CANPidController.setIZone(KIZ_DRIVE);
        CANPidController.setFF(KFF_DRIVE);
        CANPidController.setOutputRange(K_MIN_OUTPUT, K_MAX_OUTPUT);
*/
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new DriveTank());

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
    public void drive(double leftSpeed, double rightSpeed) {
        SmartDashboard.putNumber("DriveTrain_leftSpeed", leftSpeed);
        SmartDashboard.putNumber("DriveTrain_rightSpeed", rightSpeed);
        frontLeftDrive.set(leftSpeed);
        frontRightDrive.set(rightSpeed);
    }

    public void stop() {
        drive(0, 0);
    }

    public static AHRS getGyro() {
        return ahrs;
    }

    public double getGyroYaw() {
        return ahrs.getYaw();
    }

    public static double getGyroPID() {
        return ahrs.pidGet();
    }

    public static double getEncoderPosition(CANEncoder encoder) {
        return encoder.getPosition();
    }

    public static double getVelocity(CANEncoder encoder) {
        return encoder.getVelocity();
    }
    /*
     * public void setPercentVBus() { frontLeftDrive.set(ControlMode.PercentOutput,
     * 0); frontRightDrive.set(ControlMode.PercentOutput, 0); }
     * 
     * public void initSpeedMode() { frontLeftDrive.set(ControlMode.Velocity, 0);
     * frontRightDrive.set(ControlMode.Velocity, 0);
     * 
     * // Assigned PID constants to the motors. frontLeftDrive.config_kP(1,
     * SPEED_P_CONSTANT, TIMEOUT_MS); frontLeftDrive.config_kI(1, SPEED_I_CONSTANT,
     * TIMEOUT_MS); frontLeftDrive.config_kD(1, SPEED_D_CONSTANT, TIMEOUT_MS);
     * frontLeftDrive.config_kF(1, SPEED_F_CONSTANT, TIMEOUT_MS);
     * 
     * frontRightDrive.config_kP(1, SPEED_P_CONSTANT, TIMEOUT_MS);
     * frontRightDrive.config_kI(1, SPEED_I_CONSTANT, TIMEOUT_MS);
     * frontRightDrive.config_kD(1, SPEED_D_CONSTANT, TIMEOUT_MS);
     * frontRightDrive.config_kF(1, SPEED_F_CONSTANT, TIMEOUT_MS);
     * 
     * }
     * 
     * public void setControlMode(ControlMode mode) { frontLeftDrive.set(mode, 0);
     * frontRightDrive.set(mode, 0); }
     */

    public void setPercentSpeedPID(double setSpeedL, double setSpeedR) {
        setSpeedR = MathUtil.clamp(setSpeedR, -1, 1);
        setSpeedL = MathUtil.clamp(setSpeedL, -1, 1);
        frontLeftDrive.set(MAX_TICKS_PER_SECOND * setSpeedL);
        frontRightDrive.set(MAX_TICKS_PER_SECOND * setSpeedR);
    }
}
