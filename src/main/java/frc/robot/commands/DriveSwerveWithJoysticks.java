package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class DriveSwerveWithJoysticks extends Command {
    private double x, y, z;

    public DriveSwerveWithJoysticks() {
        this.requires(Robot.swerveDrive);
    }

    protected void execute() { // Order 66
        /*
         * x = SmartDashboard.getNumber("x", 0); y = SmartDashboard.getNumber("y", 0); z
         * = SmartDashboard.getNumber("z", 0);
         */

        x = OI.joystickXY.getX();
        y = OI.joystickXY.getY();
        z = OI.joystickZ.getX();

        Robot.swerveDrive.set(Math.abs(x) > 0.1 ? x : 0, Math.abs(y) > 0.1 ? y : 0, Math.abs(z) > 0.08 ? z : 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.swerveDrive.stop();
    }
}
