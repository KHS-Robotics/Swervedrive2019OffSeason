/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class SwerveDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public SwerveModule swerveModuleFrontRight, swerveModuleFrontLeft, swerveModuleRearRight, swerveModuleRearLeft;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public SwerveDrive() {
    swerveModuleFrontRight = new SwerveModule(
      RobotMap.FRONT_RIGHT_PIVOT, 
      RobotMap.FRONT_RIGHT_DRIVE, 
      RobotMap.FRONT_RIGHT_ANALOG,
      Constants.FRONT_RIGHT_P, 
      Constants.FRONT_RIGHT_I, 
      Constants.FRONT_RIGHT_D
    );
    swerveModuleFrontLeft = new SwerveModule(
      RobotMap.FRONT_LEFT_PIVOT, 
      RobotMap.FRONT_LEFT_DRIVE, 
      RobotMap.FRONT_LEFT_ANALOG,
      Constants.FRONT_LEFT_P, 
      Constants.FRONT_LEFT_I, 
      Constants.FRONT_LEFT_D
    );
    swerveModuleRearRight = new SwerveModule(
      RobotMap.REAR_RIGHT_PIVOT, 
      RobotMap.REAR_RIGHT_DRIVE, 
      RobotMap.REAR_RIGHT_ANALOG,
      Constants.REAR_RIGHT_P, 
      Constants.REAR_RIGHT_I, 
      Constants.REAR_RIGHT_D
    );
    swerveModuleRearLeft = new SwerveModule(
      RobotMap.REAR_LEFT_PIVOT, 
      RobotMap.REAR_LEFT_DRIVE, 
      RobotMap.REAR_LEFT_ANALOG,
      Constants.REAR_LEFT_P, 
      Constants.REAR_LEFT_I, 
      Constants.REAR_LEFT_D

    );
  }

  public void set(double x, double y, double z) {

  }

  public void stop() {
    set(0,0,0);
  }
}
