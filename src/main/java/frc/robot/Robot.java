/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Limelight.LightMode;
import frc.robot.commands.HoldAngle;
import frc.robot.RobotContainer;

public class Robot extends TimedRobot {
  public static RobotContainer container;

  @Override
  public void robotInit() {
    Limelight.setLedMode(LightMode.eOff);
    container = new RobotContainer();
    RobotContainer.swerveDrive.setDefaultCommand(new HoldAngle());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousPeriodic() {
    //driveWithJoystick(false);
    RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {

    if(RobotContainer.xboxController.getXButtonPressed()) {
      if(Limelight.isLedOn()) {
        Limelight.setLedMode(LightMode.eOff);
      } else {
        Limelight.setLedMode(LightMode.eOn); 
      }
    }
    
    if (RobotContainer.xboxController.getStartButton()) {
      RobotContainer.swerveDrive.resetNavx();
    }
  }

  @Override
  public void disabledInit() {
    Limelight.setLedMode(LightMode.eOff);
  }

  @Override
  public void disabledPeriodic() {
    Limelight.setLedMode(LightMode.eOff);
  }

}