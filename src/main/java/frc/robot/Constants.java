/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int CONTROLLER = 2;
  
    public static final int DRIVE_LEFT_FRONT_TALON = 10;
    public static final int DRIVE_LEFT_REAR_TALON = 11;
    public static final int DRIVE_RIGHT_FRONT_TALON = 12;
    public static final int DRIVE_RIGHT_REAR_TALON = 13;
    public static final int STEER_LEFT_FRONT_TALON = 16;
    public static final int STEER_LEFT_REAR_TALON = 17;
    public static final int STEER_RIGHT_FRONT_TALON = 18;
    public static final int STEER_RIGHT_REAR_TALON = 19;
    
    public static AHRS navX; 
    public static double zeroHeading;
    public static double zeroAngle;
    
}
