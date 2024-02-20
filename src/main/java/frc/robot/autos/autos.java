// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Storage;
import frc.robot.subsystems.Swerve;

public class autos extends Command {

  private Swerve m_Swerve;

  private final PID turnPID = new PID(0.012, 0, 0, 0, 0);

  private double currentAngle;
  private double goalAngle;

  public autos(Swerve m_Swerve, double goalAngle) {
    this.m_Swerve = m_Swerve;
    addRequirements(m_Swerve);
    this.goalAngle = goalAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = m_Swerve.getYaw().getDegrees();
    m_Swerve.drive(
    new Translation2d(0, 0), 
    turnPID.calculate(goalAngle - currentAngle) * Constants.Swerve.maxAngularVelocity, 
    false, 
    true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
