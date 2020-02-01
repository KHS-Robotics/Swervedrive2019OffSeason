/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveSwerveWithXbox;
import frc.robot.commands.HoldAngle;
import frc.robot.commands.RotateToAngleWhileDriving;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static AHRS navx = new AHRS();
  public static SwerveDrive swerveDrive = new SwerveDrive();
  public static XboxController xboxController = new XboxController(RobotMap.XBOX_PORT);

  public static DriveSwerveWithXbox driveSwerveWithXbox = new DriveSwerveWithXbox();
  private RotateToAngleWhileDriving rotateToAngleWhileDriving = new RotateToAngleWhileDriving(90);
  private HoldAngle holdAngleWhileDriving = new HoldAngle();

  JoystickButton rotateToAngle, holdAngle;

  // The robot's subsystems and commands are defined here...

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    rotateToAngle = new JoystickButton(xboxController, XboxController.Button.kY.value);
    rotateToAngle.whenHeld(rotateToAngleWhileDriving);

    holdAngle = new JoystickButton(xboxController, XboxController.Button.kA.value);
    holdAngle.whenHeld(holdAngleWhileDriving);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}