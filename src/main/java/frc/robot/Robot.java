/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveSwerveWithXbox;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends TimedRobot {
  public final static AHRS navx = new AHRS();
  public final static SwerveDrive swerveDrive = new SwerveDrive();

  @Override
  public void robotInit() {
    swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousPeriodic() {
    //driveWithJoystick(false);
    swerveDrive.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    //driveWithJoystick(false);
  }

  
}