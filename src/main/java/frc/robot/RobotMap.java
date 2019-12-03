/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // If you are using multiple modules, make sure to define both the port
  // number and the module.
  public static final int FRONT_LEFT_PIVOT = 13;
  public static final int FRONT_RIGHT_PIVOT = 0;
  public static final int REAR_LEFT_PIVOT = 15;
  public static final int REAR_RIGHT_PIVOT = 4;
  public static final int FRONT_LEFT_DRIVE = 3;
  public static final int FRONT_RIGHT_DRIVE = 11;
  public static final int REAR_LEFT_DRIVE = 12;
  public static final int REAR_RIGHT_DRIVE = 14;

  public static final int FRONT_RIGHT_ANALOG = 0;
  public static final int FRONT_LEFT_ANALOG = 1;
  public static final int REAR_RIGHT_ANALOG = 2;
  public static final int REAR_LEFT_ANALOG = 3;
}
