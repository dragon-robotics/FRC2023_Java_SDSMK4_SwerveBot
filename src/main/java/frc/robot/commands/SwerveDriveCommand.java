// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwerveDriveCommand extends CommandBase {

  private final DrivetrainSubsystem m_drivetrain;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

  public SwerveDriveCommand(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier
  ) {
      m_drivetrain = drivetrain;
      m_translationXSupplier = translationXSupplier;
      m_translationYSupplier = translationYSupplier;
      m_rotationSupplier = rotationSupplier;

      addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationXPercent = m_translationXSupplier.getAsDouble();
    double translationYPercent = m_translationYSupplier.getAsDouble();
    double rotationPercent = m_rotationSupplier.getAsDouble();

    m_drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            translationYPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            m_drivetrain.getRotation()
        )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    m_drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
