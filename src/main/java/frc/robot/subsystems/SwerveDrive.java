/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveSwerveWithJoysticks;
import frc.robot.commands.DriveSwerveWithXbox;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class SwerveDrive extends Subsystem implements PIDOutput {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public SwerveModule swerveModuleFrontRight, swerveModuleFrontLeft, swerveModuleRearRight, swerveModuleRearLeft;
  private final double w = 25.75, l = 21, r = Math.sqrt((l * l) + (w * w)), L_OVER_R = l / r, W_OVER_R = w / r;
  // MIN_X = 0.05, MIN_Y = 0.075, MIN_Z = 0.05
  private double a, b, c, d, C_D_Error = 0.19, A_B_Error = 0.19;
  private boolean fieldOriented, isHoldingAngle = false;
  private double frAngle, flAngle, rrAngle, rlAngle;
  private PIDController targetPid;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveSwerveWithXbox());
    // setDefaultCommand(new DriveSwerveWithJoysticks());
  }

  public SwerveDrive() {
    swerveModuleFrontRight = new SwerveModule(RobotMap.FRONT_RIGHT_PIVOT, RobotMap.FRONT_RIGHT_DRIVE,
        RobotMap.FRONT_RIGHT_ANALOG, Constants.FRONT_RIGHT_P, Constants.FRONT_RIGHT_I, Constants.FRONT_RIGHT_D);
    swerveModuleFrontLeft = new SwerveModule(RobotMap.FRONT_LEFT_PIVOT, RobotMap.FRONT_LEFT_DRIVE,
        RobotMap.FRONT_LEFT_ANALOG, Constants.FRONT_LEFT_P, Constants.FRONT_LEFT_I, Constants.FRONT_LEFT_D, true);
    swerveModuleRearRight = new SwerveModule(RobotMap.REAR_RIGHT_PIVOT, RobotMap.REAR_RIGHT_DRIVE,
        RobotMap.REAR_RIGHT_ANALOG, Constants.REAR_RIGHT_P, Constants.REAR_RIGHT_I, Constants.REAR_RIGHT_D);
    swerveModuleRearLeft = new SwerveModule(RobotMap.REAR_LEFT_PIVOT, RobotMap.REAR_LEFT_DRIVE,
        RobotMap.REAR_LEFT_ANALOG, Constants.REAR_LEFT_P, Constants.REAR_LEFT_I, Constants.REAR_LEFT_D, true);

    frAngle = swerveModuleFrontRight.getAngle();
    flAngle = swerveModuleFrontLeft.getAngle();
    rrAngle = swerveModuleRearRight.getAngle();
    rlAngle = swerveModuleRearLeft.getAngle();

    SmartDashboard.putNumber("A-B Error", A_B_Error);
    SmartDashboard.putNumber("C-D Error", C_D_Error);

    targetPid = new PIDController(Constants.TARGET_P, Constants.TARGET_I, Constants.TARGET_D, Robot.navx, this);

    targetPid.setInputRange(-180, 180);
    targetPid.setOutputRange(-1, 1);
    targetPid.setContinuous();
    targetPid.setAbsoluteTolerance(0.5);
    targetPid.setPIDSourceType(PIDSourceType.kDisplacement);
  }

  @Override
  public void pidWrite(double output) {
    this.set(0, 0, output);
  }

  public void disablePID() {
    if (targetPid.isEnabled()) {
      targetPid.disable();
    }
  }

  public void enablePID() {
    targetPid.enable();
  }

  public void setFOD(boolean fod) {
    fieldOriented = fod;
  }

  public void resetNavx() {
    Robot.navx.reset();
  }

  public void rotateToAngleInPlace(double setAngle) {
    targetPid.setSetpoint(setAngle);
    this.enablePID();

    /*
     * double currentAngle = Robot.navx.getYaw();
     * 
     * double zVal = 0.5;
     * 
     * if (currentAngle > setAngle) { zVal = -0.5; while (currentAngle - setAngle >
     * 2) { Robot.swerveDrive.set(0, 0, zVal); currentAngle = Robot.navx.getYaw(); }
     * } else { while (currentAngle - setAngle < -2) { Robot.swerveDrive.set(0, 0,
     * zVal); currentAngle = Robot.navx.getYaw(); } }
     */

  }

  public void set(double x, double y, double z) {
    // Setup

    y = -y;
    // Sets front to actual front

    if (fieldOriented) {
      double angle = Robot.navx.getAngle() * Math.PI / 180.0;
      SmartDashboard.putNumber("Angle (Navx)", Robot.navx.getYaw());
      double temp = y * Math.cos(angle) + x * Math.sin(angle);
      x = -y * Math.sin(angle) + x * Math.cos(angle);
      y = temp;
    }

    a = x - (z * L_OVER_R);
    b = x + (z * L_OVER_R);
    c = y - (z * W_OVER_R);
    d = y + (z * W_OVER_R);

    A_B_Error = SmartDashboard.getNumber("A-B Error", A_B_Error);
    C_D_Error = SmartDashboard.getNumber("C-D Error", C_D_Error);

    if (Math.abs(c) < C_D_Error && Math.abs(a) < A_B_Error) {
      a = b = c = d = 0;
    }

    SmartDashboard.putNumber("A", a);
    SmartDashboard.putNumber("B", b);
    SmartDashboard.putNumber("C", c);
    SmartDashboard.putNumber("D", d);

    // if (Math.abs(x) < MIN_X && Math.abs(y) < MIN_Y && Math.abs(z) < MIN_Z) {
    // swerveModuleFrontLeft.set(0, swerveModuleFrontLeft.getAngle());
    // swerveModuleFrontRight.set(0, swerveModuleFrontRight.getAngle());
    // swerveModuleRearRight.set(0, swerveModuleRearRight.getAngle());
    // swerveModuleRearLeft.set(0, swerveModuleRearLeft.getAngle());

    // Motor 1 (b,c)
    double frPivot = Math.atan2(b, c) * 180 / Math.PI + 180;
    double frSpeed = Math.sqrt(b * b + c * c);

    // Motor 2 (b,d)
    double flPivot = Math.atan2(b, d) * 180 / Math.PI + 180;
    double flSpeed = Math.sqrt(b * b + d * d);

    // Motor 3 (a,c)
    double rrPivot = Math.atan2(a, c) * 180 / Math.PI + 180;
    double rrSpeed = Math.sqrt(a * a + d * d);

    // Motor 4 (a,c)
    double rlPivot = Math.atan2(a, d) * 180 / Math.PI + 180;
    double rlSpeed = Math.sqrt(a * a + c * c);

    if (((a == 0 && Math.abs(y) < 0.005) || (Math.abs(x) < 0.005 && Math.abs(y) < 0.005 && Math.abs(z) < 0.005))
        && !isHoldingAngle) {
      frPivot = swerveModuleFrontRight.getAngle();
      flPivot = swerveModuleFrontLeft.getAngle();
      rrPivot = swerveModuleRearRight.getAngle();
      rlPivot = swerveModuleRearLeft.getAngle();

      frSpeed = 0;
      flSpeed = 0;
      rrSpeed = 0;
      rlSpeed = 0;

      isHoldingAngle = true;
    } else if (a == 0) {
      if (y > 0.02) {
        frPivot = flPivot = rrPivot = rlPivot = 180;
        isHoldingAngle = false;
      } else if (y < -0.15) {
        frPivot = flPivot = rrPivot = rlPivot = 0;
        isHoldingAngle = false;
      } else {
        if (!isHoldingAngle) {
          frPivot = swerveModuleFrontRight.getAngle();
          flPivot = swerveModuleFrontLeft.getAngle();
          rrPivot = swerveModuleRearRight.getAngle();
          rlPivot = swerveModuleRearLeft.getAngle();

          frSpeed = 0;
          flSpeed = 0;
          rrSpeed = 0;
          rlSpeed = 0;

          isHoldingAngle = true;
        }
      }
    } else {
      isHoldingAngle = false;
    }

    double max = Math.abs(frSpeed);
    if (Math.abs(flSpeed) > max)
      max = Math.abs(flSpeed);
    if (Math.abs(rlSpeed) > max)
      max = Math.abs(rlSpeed);
    if (Math.abs(rrSpeed) > max)
      max = Math.abs(rrSpeed);

    if (max > 1) {
      frSpeed /= max;
      flSpeed /= max;
      rlSpeed /= max;
      rrSpeed /= max;
    }

    swerveModuleFrontRight.setDrive(frSpeed);
    swerveModuleFrontLeft.setDrive(flSpeed);
    swerveModuleRearRight.setDrive(rrSpeed);
    swerveModuleRearLeft.setDrive(rlSpeed);

    if (!isHoldingAngle) {
      swerveModuleFrontRight.setPivot(frPivot);
      swerveModuleFrontLeft.setPivot(flPivot);
      swerveModuleRearRight.setPivot(rrPivot);
      swerveModuleRearLeft.setPivot(rlPivot);
    }

    SmartDashboard.putNumber("FR Angle", swerveModuleFrontRight.getAngle());
    SmartDashboard.putNumber("FR Voltage", swerveModuleFrontRight.getAngleVoltage());
    SmartDashboard.putNumber("FR Setpoint", frPivot);

    SmartDashboard.putNumber("FL Angle", swerveModuleFrontLeft.getAngle());
    SmartDashboard.putNumber("FL Voltage", swerveModuleFrontLeft.getAngleVoltage());
    SmartDashboard.putNumber("FL Setpoint", flPivot);

    SmartDashboard.putNumber("RR Angle", swerveModuleRearRight.getAngle());
    SmartDashboard.putNumber("RR Voltage", swerveModuleRearRight.getAngleVoltage());
    SmartDashboard.putNumber("RR Setpoint", rrPivot);

    SmartDashboard.putNumber("RL Angle", swerveModuleRearLeft.getAngle());
    SmartDashboard.putNumber("RL Voltage", swerveModuleRearLeft.getAngleVoltage());
    SmartDashboard.putNumber("RL Setpoint", rlPivot);
  }

  public void stop() {
    set(0, 0, 0);
  }
}
