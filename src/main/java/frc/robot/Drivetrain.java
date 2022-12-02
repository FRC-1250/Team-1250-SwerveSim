// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d frontLeftModuleLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightModuleLocation = new Translation2d(0.381, -0.381);
  private final Translation2d backLeftModuleLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d backRightModuleLocation = new Translation2d(-0.381, -0.381);

  // https://docs.ctre-phoenix.com/en/stable/ch21_Errata.html#talon-fx-remote-filter-device-id-must-be-15-or-less
  public final SwerveModuleTalonFX frontLeftModule = new SwerveModuleTalonFX(21, 22, 15);
  public final SwerveModuleTalonFX frontRightModule = new SwerveModuleTalonFX(4, 5, 6);
  public final SwerveModuleTalonFX backLeftModule = new SwerveModuleTalonFX(7, 8, 9);
  public final SwerveModuleTalonFX backRightModule = new SwerveModuleTalonFX(10, 11, 12);

  private final WPI_Pigeon2 pidgey = new WPI_Pigeon2(13);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftModuleLocation, frontRightModuleLocation, backLeftModuleLocation, backRightModuleLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, pidgey.getRotation2d());

  public Drivetrain() {
    pidgey.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotation      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
    ChassisSpeeds speeds;

    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, pidgey.getRotation2d());
    } else {
      speeds = new ChassisSpeeds(xSpeed, ySpeed, rotation);
    }

    var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    for (int i = 0; i < swerveModuleStates.length; i++) {
      SmartDashboard.putString("2. Desaturated SwerveModuleState " + i, swerveModuleStates[i].toString());
    }

    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    backLeftModule.setDesiredState(swerveModuleStates[2]);
    backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        pidgey.getRotation2d(),
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState());
  }
}