// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class OpenLoopWrist extends CommandBase {
  /** Creates a new OpenLoopWrist. */
  private final IntakeSubsystem mIntake;
  private double mVoltage;
  public OpenLoopWrist(IntakeSubsystem intake, double voltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    mVoltage = voltage;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.setWristVoltage(mVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.setWristVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
