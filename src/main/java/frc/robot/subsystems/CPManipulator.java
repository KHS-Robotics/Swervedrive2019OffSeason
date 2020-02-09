/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.vision.ColorBlock;
import frc.robot.vision.ColorWheel;
import frc.robot.vision.PixyCam;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

import java.util.ArrayList;

//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANPIDController;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.ControlType;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CPManipulator extends SubsystemBase {

  /**
   * Spin specified number of times
   */

  // private CANSparkMax motor;
  // private CANEncoder motorEnc;
  // private CANPIDController motorPid;
  private double speed;
  //private Solenoid solenoid;
  private int currentColorSignature;
  private ColorBlock currentBlock;
  private int initialColor;
  private double curPos;
  private double curRPM;

  public CPManipulator() {
    //solenoid = new Solenoid(RobotMap.CP_SOLONOID);
    // motor = new CANSparkMax(RobotMap.MANIPULATOR, MotorType.kBrushless);
    // motorEnc = motor.getEncoder();
    // motorPid = motor.getPIDController();

    setPosition(false);

    var tab = Shuffleboard.getTab("CP Manipulator");
    //tab.addNumber("Speed", this::getSpeed);
    //tab.addBoolean("Piston Up", solenoid::get);

    tab.addNumber("Current Color", () -> currentColorSignature);
    tab.addNumber("Dist", this::distToCenter);
    tab.addNumber("X Dist", this::xDist);
    tab.addNumber("X coord", this::centerX);
    tab.addNumber("Y coord", this::centerY);
    tab.addNumber("Initial Color", () -> initialColor);

    // motorPid.setP(0.0);
    // motorPid.setI(0.0);
    // motorPid.setD(0.0);
    // motorPid.setIZone(0.0);
  }

  @Override
  public void periodic() {
    currentBlock = getCurBlock();
    currentColorSignature = getCurColor();
    // curPos = motorEnc.getPosition();
    // curRPM = motorEnc.getVelocity();
  }

  public int distToColor(char curColor, char toColor) {
    int dist, curIndex, toIndex;

    curIndex = ColorWheel.toColor(curColor).signature;
    toIndex = ColorWheel.toColor(toColor).signature;

    dist = toIndex + curIndex;
    dist %= 4;

    if (dist == 3) {
      dist -= 4;
    }

    return dist;
  }

  public double distToCenter() {
    if(currentBlock != null) {
      return currentBlock.getDist();
    } else {
      return 0;
    }
  }

  public double xDist() {
    if(currentBlock!= null) {
      return currentBlock.getXDist();
    } else {
      return 0;
    }
  }

  public double centerX() {
    if(currentBlock != null) {
      return currentBlock.getX();
    } else {
      return 0;
    }
  }

  public double centerY() {
    if(currentBlock != null) {
      return currentBlock.getY();
    } else {
      return 0;
    }
  }

  public double distToDegrees(int dist) {
    double spins = dist / 8.0;

    return spins * 360;
  }

  public int getInitialColor() {
    return initialColor;
  }

  public void updateInitColor(int sig) {
    initialColor = sig;
  }

  public int getSensorColor(int curSig) {
    return curSig + 2;
  }

  public double degreesToColor(char curColor, char toColor) {
    return distToDegrees(distToColor(curColor, toColor));
  }

  public double spinsToRadians(double spins) {
    double degrees = spins * 360;
    double radians = (degrees * Math.PI) / 180;
    return radians;
  }

  public void spin(double speed) {
    // motorPid.setReference(speed, ControlType.kSmartMotion);
    this.speed = speed;
  }

  public static char getGameColor() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
        return 'B';
      case 'G':
        return 'G';
      case 'R':
        return 'R';
      case 'Y':
        return 'Y';
      default:
        return 'C';
      }
    } else {
      return ' ';
    }
  }

  public int getCurColor() {
    int curColorSig = -1;

    if(currentBlock != null) {
      curColorSig = currentBlock.getSig();
    }

    return curColorSig;
  }

  public ColorBlock getCurBlock() {
    ColorBlock block = null;
    ArrayList<ColorBlock> blocks = PixyCam.getBlocks();

    if (blocks.size() > 0) {
      blocks = PixyCam.sortByCenter(blocks);

      block = blocks.get(0);
    }

    return block;
  }

  public double getSpeed() {
    return speed;
  }

  public void setPosition(boolean up) {
    //solenoid.set(up);
  }
}