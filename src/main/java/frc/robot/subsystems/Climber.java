/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
//import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private CommandScheduler scheduler;
    public static TalonSRX ClimberTalon;
    public static Servo ClimberServo;
    private double ClimbSpeed;

    /**
     * 
     */
    public Climber () {
        scheduler = CommandScheduler.getInstance();
        scheduler.registerSubsystem(this);

        ClimberTalon = new TalonSRX(Constants.CLIMBER);
        ClimberTalon.configFactoryDefault();

        ClimbSpeed = 0.6;
    }

    public void extend(){
        ClimberTalon.set(ControlMode.PercentOutput, 0.6);
    }

    public void retract(){
        ClimberTalon.set(ControlMode.PercentOutput, -(0.6));
    }

    public void stop(){
        ClimberTalon.set(ControlMode.PercentOutput, 0.0);
    }
}