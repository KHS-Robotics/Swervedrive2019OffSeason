/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class HoldAngle extends CommandBase {
  private double angle;
  /**
   * Creates a new RotateToAngle.
   */
  public HoldAngle() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Init");
    angle = -RobotContainer.navx.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xSpeed = -RobotContainer.xboxController.getY(Hand.kLeft) * SwerveDrive.kMaxSpeed;
    if (Math.abs(xSpeed) < 0.17) {
      xSpeed = 0;
    }

    var ySpeed = -RobotContainer.xboxController.getX(Hand.kLeft) * SwerveDrive.kMaxSpeed;

    RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, ySpeed, angle, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
	return false;
  }
}
