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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class Limelight {
  public static NetworkTableInstance table = null;

  public static enum LightMode {
		iOn, iOff, iBlink
  }
  
  public static boolean isTarget() {
		return getValue("tv").getDouble(0) == 1;
  }
  
  public static double getTx() {
		return getValue("tx").getDouble(0.00);
  }

  public static double getTy() {
		return getValue("ty").getDouble(0.00);
	}

	public static double getTa() {
		return getValue("ta").getDouble(0.00);
	}

	public static double getTs() {
		return getValue("ts").getDouble(0.00);
	}

	public static double getTl() {
		return getValue("tl").getDouble(0.00);
	}

	public static void setLedMode(LightMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	public static void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

  private static NetworkTableEntry getValue(String key) {
		if (table == null) { //If there isn't a table, make one
			table = NetworkTableInstance.getDefault();
		}

    //return the entry
		return table.getTable("limelight").getEntry(key);
	}

 
}
