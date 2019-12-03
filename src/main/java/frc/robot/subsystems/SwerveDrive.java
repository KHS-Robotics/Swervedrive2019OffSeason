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

  private SwerveModule swerveModuleFrontRight, swerveModuleFrontLeft, swerveModuleRearRight, swerveModuleRearLeft;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public SwerveDrive() {
    swerveModuleFrontRight = new SwerveModule(
      RobotMap.FRONT_RIGHT_PIVOT, 
      RobotMap.FRONT_RIGHT_DRIVE, 
      RobotMap.FRONT_RIGHT_ANALOG
    );
    swerveModuleFrontLeft = new SwerveModule(
      RobotMap.FRONT_LEFT_PIVOT, 
      RobotMap.FRONT_LEFT_DRIVE, 
      RobotMap.FRONT_LEFT_ANALOG
    );
    swerveModuleRearRight = new SwerveModule(
      RobotMap.REAR_RIGHT_PIVOT, 
      RobotMap.REAR_RIGHT_DRIVE, 
      RobotMap.REAR_RIGHT_ANALOG
    );
    swerveModuleRearLeft = new SwerveModule(
      RobotMap.REAR_LEFT_PIVOT, 
      RobotMap.REAR_LEFT_DRIVE, 
      RobotMap.REAR_LEFT_ANALOG
    );
  }
}
