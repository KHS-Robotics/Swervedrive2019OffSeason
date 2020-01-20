/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.Robot;

public class RotateToAngle extends Command {

  public RotateToAngle() {
    super(1.5);
    this.requires(Robot.swerveDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {  
    Robot.swerveDrive.stop();
    Robot.swerveDrive.setFOD(false);
    //Robot.swerveDrive.rotateToAngleInPlace(90.0);
    Robot.swerveDrive.rotateToTargetInPlace();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Tx", Limelight.getTx());

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.swerveDrive.atSetpoint() || this.isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
