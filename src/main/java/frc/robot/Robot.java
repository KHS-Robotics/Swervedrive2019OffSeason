/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends TimedRobot {
  public final static SwerveDrive swerveDrive = new SwerveDrive();
  public final static AHRS navx = new AHRS();

  @Override
  public void robotPeriodic() {
    
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