/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import org.usfirst.frc103.Robot2020.Robot;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake extends SubsystemBase {
  public final int INTERVAL_OF_STOPS = 100;
  public double UntilStop;
  public VictorSPX intakeMotor;

  private CommandScheduler scheduler;


  /**
   * Creates a new ExampleSubsystem.
   */
  public Intake () {

    scheduler = CommandScheduler.getInstance();
    scheduler.registerSubsystem(this);
    
    intakeMotor = new VictorSPX(Constants.INTAKE_VICTOR);
    intakeMotor.configFactoryDefault();
  }

  public void runIntake(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopIntake() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

