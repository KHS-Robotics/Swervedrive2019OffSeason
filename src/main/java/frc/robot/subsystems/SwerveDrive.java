/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SwerveDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  int[] motorPorts = { RobotMap.FRONT_LEFT_PIVOT, RobotMap.FRONT_RIGHT_PIVOT, RobotMap.REAR_LEFT_PIVOT,
      RobotMap.REAR_RIGHT_PIVOT, RobotMap.FRONT_LEFT_DRIVE, RobotMap.FRONT_RIGHT_DRIVE, RobotMap.REAR_LEFT_DRIVE,
      RobotMap.REAR_RIGHT_DRIVE };

  int[] analogPorts = { RobotMap.FRONT_LEFT_ANOLOG, RobotMap.FRONT_RIGHT_ANOLOG, RobotMap.REAR_LEFT_ANOLOG,
      RobotMap.REAR_RIGHT_ANOLOG };

  SwerveModule[] swerveModules = new  SwerveModule[4];

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void init() {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i] = new SwerveModule();
      swerveModules[i].initSet(motorPorts[i], motorPorts[i + 4], analogPorts[i]);
    }
  }
}
