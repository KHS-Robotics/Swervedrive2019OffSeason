/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.PivotPIDTuner;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Limelight.LightMode;
import frc.robot.logging.Logger;
import frc.robot.subsystems.SwerveDrive;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import edu.wpi.first.wpilibj.AnalogInput;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  public static OI m_oi;
  public static SwerveDrive swerveDrive;
  public static AHRS navx;
  public static Pixy2 pixy;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  @Override
  public void robotInit() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init(); // Initializes the camera and prepares to send/receive data

    navx = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);
    swerveDrive = new SwerveDrive();
    m_oi = new OI();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {

  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */

  @Override
  public void disabledInit() {
    Limelight.setLedMode(LightMode.eOff);
    // pixy.setLamp((byte) 0, (byte) 0);
  }

  int counter = 0;

  @Override
  public void disabledPeriodic() {
    if(OI.xboxController.getYButtonPressed()) {
      if(Limelight.isLedOn()) {
        Limelight.setLedMode(LightMode.eOff);
      } else {
        Limelight.setLedMode(LightMode.eOn);
      }
    }

    try {
      pixy.getCCC().getBlocks(true, 255, 2);
      ArrayList<Block> blocks = pixy.getCCC().getBlocks();
      // if(counter > 6) {
      // for (Block block : blocks) {
      // System.out.println(block);
      // }
      // System.out.println("------------");
      // counter = 0;
      // }
      // counter++;
      if (blocks.size() > 1) {
        System.out.println("Saw more than one block");
        System.out.println(blocks.toString());
      }

      SmartDashboard.putBoolean("Yellow", false);
      SmartDashboard.putBoolean("Red", false);
      SmartDashboard.putBoolean("Green", false);
      SmartDashboard.putBoolean("Blue", false);

      for (Block block : blocks) {
        ColorWheel color = ColorWheel.toColor(block.getSignature());

        SmartDashboard.putBoolean("Yellow", color.isYellow() ? true : SmartDashboard.getBoolean("Yellow", false));
        SmartDashboard.putBoolean("Red", color.isRed() ? true : SmartDashboard.getBoolean("Red", false));
        SmartDashboard.putBoolean("Green", color.isGreen() ? true : SmartDashboard.getBoolean("Green", false));
        SmartDashboard.putBoolean("Blue", color.isBlue() ? true : SmartDashboard.getBoolean("Blue", false));
      }
    } catch (Throwable err) {
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */

  @Override
  public void autonomousInit() {
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */
  }

  /**
   * This function is called periodically during autonomous.
   */

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    Limelight.setLedMode(LightMode.eOn);

    // Scheduler.getInstance().add(new
    // PivotPIDTuner(swerveDrive.swerveModuleRearRight));
    // pixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
    // pixy.setLED(200, 30, 255); // Sets the RGB LED to purple
  }

  /**
   * This function is called periodically during operator control.
   */

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */

  @Override
  public void testPeriodic() {
  }
}