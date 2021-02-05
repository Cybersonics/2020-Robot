/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    private static NetworkTableInstance Table;
    private static NetworkTable CameraTable;
    private static NetworkTableEntry Yaw;
    private static NetworkTableEntry Pitch;
    private static NetworkTableEntry Roll;

    private Solenoid outterLightRing;
    private Solenoid innerLightRing;

    /**
    * Creates a new ExampleSubsystem.
    */
    public Vision () {
        Table = NetworkTableInstance.getDefault();
        Table.setServerTeam(103);
        CameraTable = Table.getTable("chameleon-vision").getSubTable("MyCamName");
        Yaw = CameraTable.getEntry("yaw");
        Pitch = CameraTable.getEntry("pitch");
        Roll = CameraTable.getEntry("roll");

        
        ShuffleboardTab tab = Shuffleboard.getTab("Turret");
        Pitch = tab.add("Vision Pitch", 0.0)
                .withPosition(0, 3)
                .withSize(2, 1)
                .getEntry();
        Roll = tab.add("Vision Roll", 0.0)
                .withPosition(0, 4)
                .withSize(2, 1)
                .getEntry();
        Yaw = tab.add("Vision Yaw", 0.0)
                .withPosition(0, 5)
                .withSize(2, 1)
                .getEntry();
        
        CameraServer.getInstance().startAutomaticCapture();

        this.outterLightRing = new Solenoid(Constants.OutterLightRingId);
        this.innerLightRing = new Solenoid(Constants.InnerLightRingId);

        CommandScheduler.getInstance().registerSubsystem(this);
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

    public void turnOnLightRings() {
        System.out.println("[Vision:subsystem] Turning Lights on");

        this.innerLightRing.set(true);
        this.outterLightRing.set(true);
    }

    
    public void turnOffLightRings() {
        System.out.println("[Vision:subsystem] Turning Lights off");

        this.innerLightRing.set(false);
        this.outterLightRing.set(false);
    }

    @Override()
    public void periodic() {
        Pitch.setDouble(Vision.getPitch());
        Roll.setDouble(Vision.getRoll());
        Yaw.setDouble(Vision.getYaw());
    }

}
