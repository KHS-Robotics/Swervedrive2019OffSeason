/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

/**
 * Class to put data to the SmartDashboard on a separate thread
 */
public class DemonDashboard {
    private DemonDashboard() {
    }

    private static boolean running;

    /**
     * Starts the <code>DemonDashboard</code> on a new thread.
     */
    public static void start() {
        if (running)
            return;

        Logger.info("Starting DemonDashboard...");
        new DemonDashboardThread().start();

        running = true;
    }

    /**
     * Stops the <code>DemonDashboard</code>
     */
    public static void stop() {
        running = false;
    }

    /**
     * The magic behind this class...
     */
    private static class DemonDashboardThread extends Thread implements Runnable {
        /**
         * Puts data to the SmartDashboard every 50ms. The data is retrieved from OI
         * 
         * @see org.usfirst.frc.team4342.robot.OI
         */
        @Override
        public void run() {
            SmartDashboard.putBoolean("DemonDashboard", true);
            
            while (running) {
                try {
                    
                } catch (Exception ex) {
                    Logger.error("DemonDashboard crashed!", ex);
                    DemonDashboard.stop();
                }
            }

            SmartDashboard.putBoolean("DemonDashboard", false);
        }
    }
}