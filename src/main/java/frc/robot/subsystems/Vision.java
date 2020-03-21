/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {

    private static NetworkTableInstance Table;
    private static NetworkTable CameraTable;
    private static NetworkTableEntry Yaw;
    private static NetworkTableEntry Pitch;
    private static NetworkTableEntry Roll;


    private CommandScheduler scheduler;
    /**
    * Creates a new ExampleSubsystem.
    */
    public Vision () {
        Table = NetworkTableInstance.getDefault();
        CameraTable = Table.getTable("chameleon-vision").getSubTable("MyCamName");
        Yaw = CameraTable.getEntry("yaw");
        Pitch = CameraTable.getEntry("pitch");
        Roll = CameraTable.getEntry("roll");

        scheduler = CommandScheduler.getInstance();
        scheduler.registerSubsystem(this);
    }

    public static double getYaw(){
        Yaw = CameraTable.getEntry("yaw");
        return Yaw.getDouble(0.0);
    }

    public static double getPitch(){
        Pitch = CameraTable.getEntry("pitch");
        return Pitch.getDouble(0.0);
    }

    public static double getRoll(){
        Roll = CameraTable.getEntry("roll");
        return Roll.getDouble(0.0);
    }

}
