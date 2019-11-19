/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class SwerveModule extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private WPI_TalonSRX drive, pivot;
  private AnalogInput ai;

  @Override
  protected void initDefaultCommand() {
    this.setDefaultCommand(null);
    
  }

  public void initSet(int pivotC, int driveC, int channel) {
    drive = new WPI_TalonSRX(driveC);
    pivot = new WPI_TalonSRX(pivotC);

    ai = new AnalogInput(channel);
  }
  
  private double map(double value, double minRange, double maxRange) {
    double muliplier = maxRange / minRange;
    
    return value * muliplier;
  }
}
