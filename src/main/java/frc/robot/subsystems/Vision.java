/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private NetworkTableEntry _yaw;
    private NetworkTableEntry _pitch;
    private NetworkTableEntry _roll;

    private final String LifeCamHD = "LifeCamHD";
  private final String defaultCameraTableName = "/chameleon-vision/" + LifeCamHD;
  private final NetworkTable _chameleonVisionTable = NetworkTableInstance.getDefault().getTable(defaultCameraTableName);

    public Vision () {
        CommandScheduler.getInstance().registerSubsystem(this);
        
        _yaw = _chameleonVisionTable.getEntry("yaw");
        _pitch = _chameleonVisionTable.getEntry("pitch");
        _roll = _chameleonVisionTable.getEntry("roll");

    }

    public double getX(){
        return _yaw.getDouble(0.0);
    }

    public double getY(){
        return _pitch.getDouble(0.0);
    }

    public double getRoll(){
        return _roll.getDouble(0.0);
    }

    @Override
    public void periodic() {
        _yaw = _chameleonVisionTable.getEntry("yaw");
        _pitch = _chameleonVisionTable.getEntry("pitch");
        _roll = _chameleonVisionTable.getEntry("roll");
    }

}
