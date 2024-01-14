// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class m_ArmSubsystem extends SubsystemBase {
  private int[] m_setPoints = {0, 10, 20};
  private int arm_ID = Constants.ArmConstants.ID;

  private final SparkMaxPIDController armController;
  private RelativeEncoder armEncoder;
  private CANSparkMax armMotor;
  Boolean atSetPoint = false;

  public m_ArmSubsystem() {
    armMotor = new CANSparkMax(arm_ID, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();
    armMotor.setIdleMode(IdleMode.kCoast);
    armMotor.burnFlash();
    armMotor.setSmartCurrentLimit(30);

    armController = armMotor.getPIDController();

    armController.setP(Constants.ArmConstants.kP);
    armController.setI(Constants.ArmConstants.kI);
    armController.setD(Constants.ArmConstants.kD);
    armController.setFF(Constants.ArmConstants.FF);

    armController.setFeedbackDevice(armEncoder);
    armController.setSmartMotionMaxAccel(Constants.ArmConstants.max_accel, 0);

    armController.setSmartMotionMinOutputVelocity(Constants.ArmConstants.min_vel, 0);
    armController.setSmartMotionMaxVelocity(Constants.ArmConstants.max_vel, 0);
    armController.setSmartMotionAllowedClosedLoopError(Constants.ArmConstants.allowed_error, 0);
  }

  public void toSetPoint(int setPoint) 
  {
    armController.setReference(m_setPoints[setPoint], CANSparkMax.ControlType.kSmartMotion);
    SmartDashboard.putString("DB/String 0", Double.toString(armController.getP()));
    SmartDashboard.putString("DB/String 1", Double.toString(m_setPoints[setPoint]));
    SmartDashboard.putString("DB/String 2", Double.toString(armEncoder.getPosition()));
    SmartDashboard.putString("DB/String 3", Double.toString(armMotor.getAppliedOutput()));


  }
  public void toManual(double speed)
  {
    armMotor.set(speed);
  }
  /* public void check vFinished()
  {
    if (atSetPoint)
    {
      armController.
    }
  } */


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
 
  /* protected void interrupted()
  {
   
  } */

}
