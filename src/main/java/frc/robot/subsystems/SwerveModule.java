/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {
  private final AnalogInput ai;

  private static final double kWheelRadius = 0.0508;
  private static double p, i, d;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final WPI_TalonSRX m_driveMotor;
  private final WPI_TalonSRX m_turningMotor;

  private final Encoder m_driveEncoder;

  private final PIDController m_turningPIDController;

  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int aiPort, double pVal, double iVal, double dVal, int encA, int encB) {
    ai = new AnalogInput(aiPort);
    m_driveMotor = new WPI_TalonSRX(driveMotorChannel);
    m_turningMotor = new WPI_TalonSRX(turningMotorChannel);

    p = pVal;
    i = iVal;
    d = dVal;

    m_turningPIDController = new PIDController(p, i, d);
    m_driveEncoder = new Encoder(encA, encB);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // // Set the distance (in this case, angle) per pulse for the turning encoder.
    // // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // // divided by the encoder resolution.
    // ai.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(ai.getAverageVoltage()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    final var driveOutput = m_drivePIDController.calculate(
        m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_turningPIDController.calculate(
        ai.getAverageVoltage(), state.angle.getRadians()
    );

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }
}