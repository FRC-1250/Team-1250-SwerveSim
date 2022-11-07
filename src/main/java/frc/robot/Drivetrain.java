// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  // https://docs.ctre-phoenix.com/en/stable/ch21_Errata.html#talon-fx-remote-filter-device-id-must-be-15-or-less
  private final SwerveModuleTalonFX fx_frontLeft = new SwerveModuleTalonFX(1, 2, 3);
  private final SwerveModuleTalonFX fx_frontRight = new SwerveModuleTalonFX(4, 5, 6);
  private final SwerveModuleTalonFX fx_backLeft = new SwerveModuleTalonFX(7, 8, 9);
  private final SwerveModuleTalonFX fx_backRight = new SwerveModuleTalonFX(10, 11, 12);

  private final Pigeon2 pidgey = new Pigeon2(13);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
      Rotation2d.fromDegrees(pidgey.getYaw()));

  public Drivetrain() {
    pidgey.setYaw(0);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds speeds;

    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(pidgey.getYaw()));
    } else {
      speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    for (int i = 0; i < swerveModuleStates.length; i++) {
      SmartDashboard.putString("2. Desaturated SwerveModuleState " + i, swerveModuleStates[i].toString());
    }

    fx_frontLeft.setDesiredState(swerveModuleStates[0]);
    fx_frontRight.setDesiredState(swerveModuleStates[1]);
    fx_backLeft.setDesiredState(swerveModuleStates[2]);
    fx_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(pidgey.getYaw()),
        fx_frontLeft.getState(),
        fx_frontRight.getState(),
        fx_backLeft.getState(),
        fx_backRight.getState());
  }

}
