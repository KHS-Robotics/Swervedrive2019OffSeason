/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSwerveWithXbox extends Command {
  public double x, y, z; 

  public DriveSwerveWithXbox() {
    this.requires(Robot.swerveDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.swerveDrive.stop();
    SmartDashboard.putNumber("x", 0);
    SmartDashboard.putNumber("y", 0);
    SmartDashboard.putNumber("z", 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { //Order 66
    /*x = SmartDashboard.getNumber("x", 0);
    y = SmartDashboard.getNumber("y", 0);
    z = SmartDashboard.getNumber("z", 0);*/

    x = OI.xboxController.getX(Hand.kLeft);
    y = OI.xboxController.getY(Hand.kLeft);
    z = OI.xboxController.getX(Hand.kRight);

    if(OI.xboxController.getStartButton()) {
      Robot.swerveDrive.resetNavx();
    }

    Robot.swerveDrive.setFOD(!OI.xboxController.getBumper(Hand.kLeft));
    
    Robot.swerveDrive.set(
      Math.abs(x) > 0.05 ? x : 0,
      Math.abs(y) > 0.05 ? y : 0,
      Math.abs(z) > 0.08 ? z : 0
    );
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.swerveDrive.stop();
  }
}
