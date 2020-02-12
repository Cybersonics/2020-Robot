/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static int INTAKE_VICTOR = 5;
    public final static int INDEXER_VICTOR_ONE = 4;
    public final static int SHOOTER_SPARK_ONE = 1;
    public final static int SHOOTER_SPARK_TWO = 2;
    public final static int PIVOT_TALON = 3;

    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public final static int XBOX_CONTROLLER = 2;
    
    public static final int DRIVE_FRONT_LEFT_STEER_ENCODER = 0;
    public static final int DRIVE_FRONT_LEFT_STEER_MOTOR = 20;
    public static final double DRIVE_FRONT_LEFT_STEER_OFFSET = -Math.toRadians(154.3);
    public static final int DRIVE_FRONT_LEFT_DRIVE_MOTOR = 10;

    public static final int DRIVE_FRONT_RIGHT_STEER_ENCODER = 2;
    public static final int DRIVE_FRONT_RIGHT_STEER_MOTOR = 22;
    public static final double DRIVE_FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(329.0);
    public static final int DRIVE_FRONT_RIGHT_DRIVE_MOTOR = 12;

    public static final int DRIVE_BACK_LEFT_STEER_ENCODER = 1;
    public static final int DRIVE_BACK_LEFT_STEER_MOTOR = 21;
    public static final double DRIVE_BACK_LEFT_STEER_OFFSET = -Math.toRadians(218.1);
    public static final int DRIVE_BACK_LEFT_DRIVE_MOTOR = 11;

    public static final int DRIVE_BACK_RIGHT_STEER_ENCODER = 3;
    public static final int DRIVE_BACK_RIGHT_STEER_MOTOR = 23;
    public static final double DRIVE_BACK_RIGHT_STEER_OFFSET = -Math.toRadians(268.9);
    public static final int DRIVE_BACK_RIGHT_DRIVE_MOTOR = 13;
    
}
