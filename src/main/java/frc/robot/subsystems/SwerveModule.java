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
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveModule {
  private final AnalogInput ai;

  private static double p, i, d;
  private static final int kEncoderResolution = 2048;

  private static final double MIN_VOLTAGE = 0.2, MAX_VOLTAGE = 4.76,
      DELTA_VOLTAGE = MAX_VOLTAGE - MIN_VOLTAGE,
      distancePerPulse = (0.0254 * 4 * Math.PI * 12 * 19) / (kEncoderResolution * 32 * 60);
      //kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed, kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared;

  private static double offset;
  private static boolean isInverted;

  private final WPI_TalonSRX m_driveMotor;
  private final WPI_TalonSRX m_turningMotor;

  private final Encoder m_driveEncoder;

  private final PIDController m_turningPIDController;

  //private final PIDController m_drivePIDController = new PIDController(1 / 10.0, 0, 0);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param aiPort              Port num of the Analog Input
   * @param turnP               P Val of Turn PID
   * @param turnI               I Val of Turn PID
   * @param TurnD               D Val of Turn PID
   * @param encA                Port of Encoder A Channel
   * @param encB                Port of Encoder B Channel
   * @param offset              Offset of the Pivot Motor, 0 by default
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int aiPort, double turnP, double turnI,
      double turnD, int encA, int encB, boolean reversed) {
    isInverted = reversed;

    ai = new AnalogInput(aiPort);
    m_driveMotor = new WPI_TalonSRX(driveMotorChannel);
    m_turningMotor = new WPI_TalonSRX(turningMotorChannel);

    p = turnP;
    i = turnI;
    d = turnD;

    m_turningPIDController = new PIDController(p, i, d);
    m_driveEncoder = new Encoder(encA, encB);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(distancePerPulse);

    // // Set the distance (in this case, angle) per pulse for the turning encoder.
    // // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // // divided by the encoder resolution.
    // ai.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int aiPort, double pVal, double iVal, double dVal, int encA, int encB) {
    this(driveMotorChannel, turningMotorChannel, aiPort, pVal, iVal, dVal, encA, encB, false);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(voltsToRadians(ai.getAverageVoltage())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public double setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    //final var driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_turningPIDController.calculate(voltsToRadians(ai.getAverageVoltage()), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(isInverted ? -state.speedMetersPerSecond : state.speedMetersPerSecond);
    m_turningMotor.set(turnOutput);

    return state.angle.getDegrees();
  }

  public static double toAngle(double voltage) {
    return ((360.0 * (voltage - MIN_VOLTAGE) / DELTA_VOLTAGE) + 360.0 - offset) % 360;
  }

  public static double voltsToRadians(double voltage) {
    return Math.toRadians(toAngle(voltage));
  }

  public static double radiansToVolts(double radians) {
    return (((Math.toDegrees(radians) - 360.0) * DELTA_VOLTAGE) / 360.0) + MIN_VOLTAGE;
  }
}