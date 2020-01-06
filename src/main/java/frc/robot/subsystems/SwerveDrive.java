/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveSwerveWithXbox;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class SwerveDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public SwerveModule swerveModuleFrontRight, swerveModuleFrontLeft, swerveModuleRearRight, swerveModuleRearLeft;
  private final double l = 25.75, w = 21, r = Math.sqrt((l * l) + (w * w)), L_OVER_R = l / r, W_OVER_R = w / r;
  private double a, b, c, d, speed;
  private boolean fieldOriented;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveSwerveWithXbox());
  }

  public SwerveDrive() {
    swerveModuleFrontRight = new SwerveModule(RobotMap.FRONT_RIGHT_PIVOT, RobotMap.FRONT_RIGHT_DRIVE,
        RobotMap.FRONT_RIGHT_ANALOG, Constants.FRONT_RIGHT_P, Constants.FRONT_RIGHT_I, Constants.FRONT_RIGHT_D);
    swerveModuleFrontLeft = new SwerveModule(RobotMap.FRONT_LEFT_PIVOT, RobotMap.FRONT_LEFT_DRIVE,
        RobotMap.FRONT_LEFT_ANALOG, Constants.FRONT_LEFT_P, Constants.FRONT_LEFT_I, Constants.FRONT_LEFT_D, true);
    swerveModuleRearRight = new SwerveModule(RobotMap.REAR_RIGHT_PIVOT, RobotMap.REAR_RIGHT_DRIVE,
        RobotMap.REAR_RIGHT_ANALOG, Constants.REAR_RIGHT_P, Constants.REAR_RIGHT_I, Constants.REAR_RIGHT_D);
    swerveModuleRearLeft = new SwerveModule(RobotMap.REAR_LEFT_PIVOT, RobotMap.REAR_LEFT_DRIVE,
        RobotMap.REAR_LEFT_ANALOG, Constants.REAR_LEFT_P, Constants.REAR_LEFT_I, Constants.REAR_LEFT_D, true);
  }

  public void setFOD(boolean fod) {
    fieldOriented = fod;
  } 

  public void set(double x, double y, double z) {
    //Setup

    y = -y;
    // Sets front to actual front

    if(fieldOriented) {
      double angle = Robot.navx.getAngle();
      double temp = y * Math.cos(angle) + x * Math.sin(angle);
      x = -y * Math.sin(angle) + x * Math.cos(angle);
      y = temp;
    }

    a = x - (z * L_OVER_R);
    b = x + (z * L_OVER_R);
    c = y - (z * W_OVER_R);
    d = y + (z * W_OVER_R);

    SmartDashboard.putNumber("A", a);
    SmartDashboard.putNumber("B", b);
    SmartDashboard.putNumber("C", c);
    SmartDashboard.putNumber("D", d);

    //Motor 1 (b,c)
    swerveModuleFrontRight.setPivot(Math.atan2(b,c) * 180/Math.PI + 180);
    speed = Math.sqrt(b*b + c*c);

    speed = speed > 1 ? 1 : speed;
    speed = speed < -1 ? -1 : speed;

    swerveModuleFrontRight.setDrive(speed);

    //Motor 2 (b,d)
    swerveModuleFrontLeft.setPivot(Math.atan2(b,d) * 180/Math.PI + 180);
    speed = Math.sqrt(b*b + d*d);

    speed = speed > 1 ? 1 : speed;
    speed = speed < -1 ? -1 : speed;

    swerveModuleFrontLeft.setDrive(speed);

    //Motor 3 (a,c)
    swerveModuleRearRight.setPivot(Math.atan2(a,c) * 180/Math.PI + 180);
    speed = Math.sqrt(a*a + d*d);

    speed = speed > 1 ? 1 : speed;
    speed = speed < -1 ? -1 : speed;

    swerveModuleRearRight.setDrive(speed);

    //Motor 4 (a,c)
    swerveModuleRearLeft.setPivot(Math.atan2(a,d) * 180/Math.PI + 180);
    speed = Math.sqrt(a*a + c*c);

    speed = speed > 1 ? 1 : speed;
    speed = speed < -1 ? -1 : speed;

    swerveModuleRearLeft.setDrive(speed);
  }

  public void stop() {
    set(0, 0, 0);
  }
}
