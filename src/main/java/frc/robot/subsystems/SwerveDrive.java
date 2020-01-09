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
import frc.robot.commands.DriveSwerveWithJoysticks;
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
  private final double l = 25.75, w = 21, r = Math.sqrt((l * l) + (w * w)), L_OVER_R = l / r, W_OVER_R = w / r,
      MIN_X = 0.05, MIN_Y = 0.075, MIN_Z = 0.05;
  private double a, b, c, d;
  private boolean fieldOriented;
  private double frAngle, flAngle, rrAngle, rlAngle;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveSwerveWithXbox());
    //setDefaultCommand(new DriveSwerveWithJoysticks());
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
    
    frAngle = swerveModuleFrontRight.getAngle();
    flAngle = swerveModuleFrontLeft.getAngle();
    rrAngle = swerveModuleRearRight.getAngle();
    rlAngle = swerveModuleRearLeft.getAngle();
  }

  public void setFOD(boolean fod) {
    fieldOriented = fod;
  }

  public void set(double x, double y, double z) {
    // Setup

    y = -y;
    // Sets front to actual front

    if (fieldOriented) {
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

    // if (Math.abs(x) < MIN_X && Math.abs(y) < MIN_Y && Math.abs(z) < MIN_Z) {
    // swerveModuleFrontLeft.set(0, swerveModuleFrontLeft.getAngle());
    // swerveModuleFrontRight.set(0, swerveModuleFrontRight.getAngle());
    // swerveModuleRearRight.set(0, swerveModuleRearRight.getAngle());
    // swerveModuleRearLeft.set(0, swerveModuleRearLeft.getAngle());

    // Motor 1 (b,c)
    double frPivot = Math.atan2(b, c) * 180 / Math.PI + 180;
    double frSpeed = Math.sqrt(b * b + c * c);
    
    // Motor 2 (b,d)
    double flPivot = Math.atan2(b, d) * 180 / Math.PI + 180;
    double flSpeed = Math.sqrt(b * b + d * d);

    // Motor 3 (a,c)
    double rrPivot = Math.atan2(a, c) * 180 / Math.PI + 180;
    double rrSpeed = Math.sqrt(a * a + d * d);

    // Motor 4 (a,c)
    double rlPivot = Math.atan2(a, d) * 180 / Math.PI + 180;
    double rlSpeed = Math.sqrt(a * a + c * c);

    if ((a == 0 && Math.abs(y) < 0.005) || (Math.abs(x) < 0.005 && Math.abs(y) < 0.005 && Math.abs(z) < 0.005)) {
      frPivot = swerveModuleFrontRight.getAngle();
      flPivot = swerveModuleFrontLeft.getAngle();
      rrPivot = swerveModuleRearRight.getAngle();
      rlPivot = swerveModuleRearLeft.getAngle();
    } else if (a == 0) {
      if (y > 0.02) {
        frPivot = flPivot = rrPivot = rlPivot = 180;
      } else if (y < -0.15) {
        frPivot = flPivot = rrPivot = rlPivot = 0;
      } else {
        frPivot = swerveModuleFrontRight.getAngle();
        flPivot = swerveModuleFrontLeft.getAngle();
        rrPivot = swerveModuleRearRight.getAngle();
        rlPivot = swerveModuleRearLeft.getAngle();
      }
    }

    double max = Math.abs(frSpeed);
		if(Math.abs(flSpeed) > max)
			max = Math.abs(flSpeed);
		if(Math.abs(rlSpeed) > max)
			max = Math.abs(rlSpeed);
		if(Math.abs(rrSpeed) > max)
			max = Math.abs(rrSpeed);
		
		if(max > 1) {
			frSpeed /= max;
			flSpeed /= max;
			rlSpeed /= max;
			rrSpeed /= max;
    }
    
    swerveModuleFrontRight.setDrive(frSpeed);
    swerveModuleFrontLeft.setDrive(flSpeed);
    swerveModuleRearRight.setDrive(rrSpeed);
    swerveModuleRearLeft.setDrive(rlSpeed);

    swerveModuleFrontRight.setPivot(frPivot);
    swerveModuleFrontLeft.setPivot(flPivot);
    swerveModuleRearRight.setPivot(rrPivot);
    swerveModuleRearLeft.setPivot(rlPivot);

  }

  public void stop() {
    set(0, 0, 0);
  }
}
