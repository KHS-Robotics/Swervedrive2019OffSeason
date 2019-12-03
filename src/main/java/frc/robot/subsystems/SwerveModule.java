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

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class SwerveModule extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static final double MIN_VOLTAGE = 0, MAX_VOLTAGE = 5, DELTA_VOLTAGE = MAX_VOLTAGE - MIN_VOLTAGE;
  private double offset;

  private WPI_TalonSRX drive, pivot;
  private AnalogInput ai;
  private PIDController pivotPID;

  @Override
  protected void initDefaultCommand() {
    this.setDefaultCommand(null);
  }

  public SwerveModule (int pivotPort, int drivePort, int aiChannel) {
    pivotPID = new PIDController(0.0, 0.0, 0.0, ai, pivot);
    
    pivotPID.setInputRange(MIN_VOLTAGE, MAX_VOLTAGE);
    pivotPID.setOutputRange(-1, 1);
    pivotPID.setContinuous();
    pivotPID.setPIDSourceType(PIDSourceType.kDisplacement);

    drive = new WPI_TalonSRX(drivePort);
    pivot = new WPI_TalonSRX(pivotPort);

    ai = new AnalogInput(aiChannel);
  }

  public void setDrive(double output) {
    drive.set(output);
  }

  public void setPivot(double angle) {
    pivotPID.setSetpoint(angle);
  }

  public double toVoltage(double angle) {
    angle %= 360;
    angle += 360;
    return (MIN_VOLTAGE + (DELTA_VOLTAGE * (offset + angle - 360))) / 360.0;
  }

  public double toAngle(double voltage) {
    return ((360.0 * (voltage - MIN_VOLTAGE) / DELTA_VOLTAGE) + 360.0 - offset) % 360;
  }
}