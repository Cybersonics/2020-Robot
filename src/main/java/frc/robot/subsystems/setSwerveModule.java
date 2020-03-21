/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class setSwerveModule extends PIDSubsystem {
  /**
   * Creates a new setSwerveModule.
   */
  private static final double STEER_P = .0005, STEER_I = 0.00, STEER_D = 0.0;
  public double currentPosition;
  private CANSparkMax steerMotor;
  private double currentAngle;

  private static final double RAMP_RATE = 0.5;

  public setSwerveModule() {
    super(
        // The PIDController used by the subsystem
		new PIDController(STEER_P, STEER_I, STEER_D));
		getController().enableContinuousInput(0, 360);
		enable();
		

  }

  @Override
  public void useOutput(double output, double setpoint) {

  }

  @Override
  public double getMeasurement() {
    return currentAngle;
  }

	public boolean atSetpoint() {
		return m_controller.atSetpoint();
	}

	public void setAngle(double curAngle) {
		getController().setSetpoint(curAngle);
	}

	public double getOutput(double targetAngle){
		return getController().calculate(targetAngle);
	}


}
