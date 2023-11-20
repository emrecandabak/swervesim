// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Rotation3dUtils;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeWristMotor = new TalonFX(40);

  private final SingleJointedArmSim wristSim = 
          new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
          35.0,
            SingleJointedArmSim.estimateMOI(0.38, 5.000),
            0.38,
              Units.degreesToRadians(0),
              Units.degreesToRadians(90),
                true,
                VecBuilder.fill(0.01 / 360.0)
                  );
  
  private final Translation3d wristRootPosition = new Translation3d(0.17,0.0,0.445);

  public IntakeSubsystem() {
      intakeWristMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumberArray("intakepos", Rotation3dUtils.concatenateArrays(new Double[] {wristRootPosition.getX(), wristRootPosition.getY(), wristRootPosition.getZ()}, Rotation3dUtils.rotation3dToDoubleArray(new Rotation3d(0,Units.degreesToRadians(getWristAngle()),0))));
    SmartDashboard.putNumber("wrangle", getWristAngle());
  }

  public void setWristVoltage(double voltage){
    intakeWristMotor.setVoltage(voltage);
  }

  public double getWristAngle(){
    return intakeWristMotor.getRotorPosition().getValue() * 360.0 / (35.0);
  }

  @Override
  public void simulationPeriodic() {

    intakeWristMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    wristSim.setInput(intakeWristMotor.getDutyCycle().getValue() * RobotController.getBatteryVoltage());

    wristSim.update(0.02);

    final double kTurnDistancePerPulse = 360.0 / (35.0);

    double angleVel = Units.radiansToDegrees(wristSim.getVelocityRadPerSec()) / kTurnDistancePerPulse;
    intakeWristMotor.getSimState().setRotorVelocity(angleVel);

    double anglePos = Units.radiansToDegrees(wristSim.getAngleRads()) / kTurnDistancePerPulse;
    intakeWristMotor.getSimState().setRawRotorPosition(anglePos);

  }
}
