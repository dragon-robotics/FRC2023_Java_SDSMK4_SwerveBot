// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  private static final double MAX_VOLTAGE = 12.0;
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private final WPI_Pigeon2 gyroscope = new WPI_Pigeon2(Constants.DRIVETRAIN_PIGEON_ID);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
  
  private final SwerveDriveOdometry odometry;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      0.0,
      0.0,
      0.0);

  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule = new MkSwerveModuleBuilder()
            .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.FRONT_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)
            .build();

    m_frontRightModule = new MkSwerveModuleBuilder()
            .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)
            .build();

    m_backLeftModule = new MkSwerveModuleBuilder()
            .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.BACK_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.BACK_LEFT_MODULE_STEER_OFFSET)
            .build();

    m_backRightModule = new MkSwerveModuleBuilder()
            .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
            .build();
    
    odometry = new SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(gyroscope.getCompassHeading()),
        new SwerveModulePosition[]{ m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition() }
    );

    shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
    shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
    shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
  }

  public void zeroGyroscope() {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyroscope.getCompassHeading()),
        new SwerveModulePosition[]{ m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition() },
        new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
    );
  }

  public Rotation2d getRotation() {
      return odometry.getPoseMeters().getRotation();
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
      this.chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
        Rotation2d.fromDegrees(gyroscope.getCompassHeading()),
        new SwerveModulePosition[]{ m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition() }
    );

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }
}
