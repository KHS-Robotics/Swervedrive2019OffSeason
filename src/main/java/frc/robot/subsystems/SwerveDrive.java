/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrive {
  public static final double kMaxSpeed = 1.0; // 1 meters per second
  public static final double kMaxAngularSpeed = 1; // 1 radian per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.327025, 0.2667);
  private final Translation2d m_frontRightLocation = new Translation2d(0.327025, -0.2667);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.327025, 0.2667);
  private final Translation2d m_backRightLocation = new Translation2d(-0.327025, -0.2667);

  private final SwerveModule m_frontLeft = new SwerveModule(
    RobotMap.FRONT_LEFT_DRIVE, 
    RobotMap.FRONT_LEFT_PIVOT,
    RobotMap.FRONT_LEFT_ANALOG, 
    Constants.FRONT_LEFT_P, 
    Constants.FRONT_LEFT_I, 
    Constants.FRONT_LEFT_D,
    RobotMap.FRONT_LEFT_DRIVE_ENC_A, 
    RobotMap.FRONT_LEFT_DRIVE_ENC_B, 
    true
  );
  private final SwerveModule m_frontRight = new SwerveModule(
    RobotMap.FRONT_RIGHT_DRIVE,
    RobotMap.FRONT_RIGHT_PIVOT,
    RobotMap.FRONT_RIGHT_ANALOG, 
    Constants.FRONT_RIGHT_P, 
    Constants.FRONT_RIGHT_I, 
    Constants.FRONT_RIGHT_D,
    RobotMap.FRONT_RIGHT_DRIVE_ENC_A, 
    RobotMap.FRONT_RIGHT_DRIVE_ENC_B, 
    true
  );
  private final SwerveModule m_backLeft = new SwerveModule(
    RobotMap.REAR_LEFT_DRIVE, 
    RobotMap.REAR_LEFT_PIVOT,
    RobotMap.REAR_LEFT_ANALOG, 
    Constants.REAR_LEFT_P, 
    Constants.REAR_LEFT_I, 
    Constants.REAR_LEFT_D,
    RobotMap.REAR_LEFT_DRIVE_ENC_A, 
    RobotMap.REAR_LEFT_DRIVE_ENC_B, 
    true
  );
  private final SwerveModule m_backRight = new SwerveModule(
    RobotMap.REAR_RIGHT_DRIVE, 
    RobotMap.REAR_RIGHT_PIVOT,
    RobotMap.REAR_RIGHT_ANALOG, 
    Constants.REAR_RIGHT_P, 
    Constants.REAR_RIGHT_I, 
    Constants.REAR_RIGHT_D,
    RobotMap.REAR_RIGHT_DRIVE_ENC_A, 
    RobotMap.REAR_RIGHT_DRIVE_ENC_B, 
    true
  );

  // private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
      m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

  public SwerveDrive() {
    Robot.navx.reset();
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-Robot.navx.getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics
      .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
        : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    SmartDashboard.putNumber("FL Angle", m_frontLeft.setDesiredState(swerveModuleStates[0]));
    SmartDashboard.putNumber("FR Angle", m_frontRight.setDesiredState(swerveModuleStates[1]));
    SmartDashboard.putNumber("RL Angle", m_backLeft.setDesiredState(swerveModuleStates[2]));
    SmartDashboard.putNumber("RR Angle", m_backRight.setDesiredState(swerveModuleStates[3]));
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(getAngle(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
        m_backRight.getState());
  }
}