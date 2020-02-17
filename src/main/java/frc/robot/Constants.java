/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    public final static int XBOX_CONTROLLER = 2;

    public final static int INTAKE_VICTOR = 5;
    public final static int INDEXER_VICTOR = 4;
    public final static int SHOOTER_SPARK_ONE = 1;
    public final static int SHOOTER_SPARK_TWO = 2;
    public final static int PIVOT_TALON = 3;
    public final static double MAX_DART_VALUE = 0.0; //needs to be updated
    public final static double MIN_DART_VALUE = 0.0; //needs to be updated
    public final static int SHOOTER_ENCODER = 0;//needs to be changed
    public final static double SHOOTER_ENCODER_ANGLE_INCREASE = 0.0; //needs to be updated // increase in angle per encoder rotation

    // Constants for Shooter
    // inches unless stated otherwise
    public final static double BALL_MASS = 0.2; //ball mass is 0.2 lb
    public final static double TOWER_HEIGHT = 84.0; //floor to the bottom of the hexagon
    public final static double HEXAGON_HEIGHT = 30.0;// Outer hexagon hole height (diameter)
    public final static double INNER_HOLE_DIAMETER = 13.0;// Diameter of inner hole
    //Difference in distance between the bottom of the inner whole and the bottom of the outer hexagon
    public final static double HEXAGON_INNER_HOLE_BOTTOM_DIF = 8.5;
    public final static double HEXAGON_SIDE = 17.0;//leng of one side of the hexagon
    public final static double LAUNCHER_HEIGHT = 40.0; //height of launcher on robot
    public final static double MAX_ANGLE = 33.0;//hieghest possible angle of shooter
    public final static double MIN_ANGLE = -4.0;//hieghest possible angle of shooter
    public final static double BEHIND_COLOR_WHEEL_ANGLE = 8.0; //angle of shooter when behind color wheel
    public final static double AUTON_START_SHOOTER_ANGLE = 26.0; //26 deg. when 10ft awy from target
    
    public static final int DRIVE_FRONT_LEFT_STEER_ENCODER = 0;
    public static final int DRIVE_FRONT_LEFT_STEER_MOTOR = 20;
    public static final double DRIVE_FRONT_LEFT_STEER_OFFSET = -Math.toRadians(132.0);
    public static final int DRIVE_FRONT_LEFT_DRIVE_MOTOR = 10;

    public static final int DRIVE_FRONT_RIGHT_STEER_ENCODER = 3;
    public static final int DRIVE_FRONT_RIGHT_STEER_MOTOR = 23;
    public static final double DRIVE_FRONT_RIGHT_STEER_OFFSET = Math.toRadians(95.0);
    public static final int DRIVE_FRONT_RIGHT_DRIVE_MOTOR = 13;

    public static final int DRIVE_BACK_LEFT_STEER_ENCODER = 1;
    public static final int DRIVE_BACK_LEFT_STEER_MOTOR = 21;
    public static final double DRIVE_BACK_LEFT_STEER_OFFSET = Math.toRadians(80.0);
    public static final int DRIVE_BACK_LEFT_DRIVE_MOTOR = 11;

    public static final int DRIVE_BACK_RIGHT_STEER_ENCODER = 2;
    public static final int DRIVE_BACK_RIGHT_STEER_MOTOR = 22;
    public static final double DRIVE_BACK_RIGHT_STEER_OFFSET = Math.toRadians(127.0); //-53
    public static final int DRIVE_BACK_RIGHT_DRIVE_MOTOR = 12;
    
}
