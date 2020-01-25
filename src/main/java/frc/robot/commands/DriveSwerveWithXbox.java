/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSwerveWithXbox extends Command {
  public double x, y, z;
  private boolean fieldRelative = false;

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

    SmartDashboard.putBoolean("wadwad", true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { // Order 66

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = -OI.xboxController.getY(GenericHID.Hand.kLeft) * SwerveDrive.kMaxSpeed;
    if (Math.abs(xSpeed) < 0.17) {
      xSpeed = 0;
    }

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = OI.xboxController.getX(GenericHID.Hand.kLeft) * SwerveDrive.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot = -OI.xboxController.getX(GenericHID.Hand.kRight) * SwerveDrive.kMaxAngularSpeed;

    Robot.swerveDrive.drive(xSpeed, ySpeed, rot, fieldRelative);

    if (OI.xboxController.getStartButton()) {
      Robot.swerveDrive.resetNavx();
    }

    /*
     * if (Math.abs(x) + Math.abs(y) + Math.abs(z) > 0.35) {
     * Robot.swerveDrive.disablePID(); }
     */

    fieldRelative = (!OI.xboxController.getBumper(Hand.kLeft));// getRawButton(ButtonMap.XboxButton.toggleFOD));

    Robot.swerveDrive.drive(Math.abs(x) > 0.05 ? x : 0, Math.abs(y) > 0.05 ? y : 0, Math.abs(z) > 0.08 ? z : 0, fieldRelative);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putBoolean("wadwad", false);
    Robot.swerveDrive.stop();
  }
}