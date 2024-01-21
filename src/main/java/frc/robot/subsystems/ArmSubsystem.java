// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.compound.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
  
  private TalonFX armMotor = new TalonFX(Constants.ArmConstants.ARM_ID);
  private int[] m_setPoints = {0, 5, 10};

  
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    armMotor.setPosition(0);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;


    slot0Configs.kV = Constants.ArmConstants.kV;
    slot0Configs.kP = Constants.ArmConstants.kP;
    slot0Configs.kI = Constants.ArmConstants.kI;
    slot0Configs.kD = Constants.ArmConstants.kD;

   /* var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0;
    motionMagicConfigs.MotionMagicExpo_kV = 0.12;
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; */

    armMotor.getConfigurator().apply(slot0Configs, 0.020);
    SmartDashboard.putString("DB/String 1", Double.toString(slot0Configs.kV));
    SmartDashboard.putString("DB/String 2", Double.toString(slot0Configs.kP));
    SmartDashboard.putString("DB/String 3", Double.toString(slot0Configs.kI));
    SmartDashboard.putString("DB/String 4", Double.toString(slot0Configs.kD));

  }

  public void toSetPoint(int setPoint) 
  {
    var motorPosSignal = armMotor.getRotorPosition();
    var motorPos = motorPosSignal.getValue();

    final MotionMagicExpoVoltage m_PIDRequest = new MotionMagicExpoVoltage(0);
    armMotor.setControl(m_PIDRequest.withPosition(m_setPoints[setPoint]));

    SmartDashboard.putString("DB/String 0", Double.toString(motorPos));
    
  }
  public void manualControl()
  {
    armMotor.set(0.2);
  }
  public void stopControl()
  {
    armMotor.set(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
