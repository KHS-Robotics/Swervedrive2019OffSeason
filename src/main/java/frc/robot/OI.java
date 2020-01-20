/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.RotateToAngle;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
  public static XboxController xboxController = new XboxController(RobotMap.XBOX_PORT);
  public static Joystick joystickXY = new Joystick(RobotMap.JOYSTICK_XY);
  public static Joystick joystickZ = new Joystick(RobotMap.JOYSTICK_Z);

  public OI() {
    JoystickButton rotateToAngle = new JoystickButton(xboxController, XboxController.Button.kY.value);
    rotateToAngle.whenPressed(new RotateToAngle());
  }
}
