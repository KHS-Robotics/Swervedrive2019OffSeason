/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.SwerveModule;

public class PivotPIDTuner extends Command {
  double p, i, d, setPoint;
  SwerveModule sModule;

  public PivotPIDTuner(SwerveModule SM) {
    this.requires(SM);
    sModule = SM;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putNumber("P-Value", 0.0);
    SmartDashboard.putNumber("I-Value", 0.0);
    SmartDashboard.putNumber("D-Value", 0.0);

    SmartDashboard.putNumber("Setpoint (Angle)", 0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    p = SmartDashboard.getNumber("P-Value", 0.0);
    i = SmartDashboard.getNumber("I-Value", 0.0);
    d = SmartDashboard.getNumber("D-Value", 0.0);

    setPoint = SmartDashboard.getNumber("Setpoint (Angle)", 0.0);

    sModule.setPid(p, i, d);
    sModule.setPivot(setPoint);

    //SmartDashboard.putNumber("AI", sModule.getAngleVoltage());
    //SmartDashboard.putNumber("Angle", sModule.getAngle());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    sModule.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
