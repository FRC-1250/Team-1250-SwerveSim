// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
  /**
   * The maxmimum velocity that the swerve modules is capable of in meters per
   * second (m/s).
   * This value can be derived mathimatically OR will be available from the
   * manufacturer of the
   * swerve module.
   * 
   * @see <a href=
   *      "https://www.swervedrivespecialties.com/products/mk4-swerve-module">MK4
   *      Swerve Module L1 - Standard </a>
   */
  public static final double maxDriveSpeed = 4.115;

  /**
   * The maxmimum angular velocity that the swerve module is capable of in
   * rotations per second.
   * This value is easy to represent as some multiple of PI.
   * <p>
   * For example: 2 * Math.PI is 1 rotation per second.
   */
  public static final double maxTurningSpeed = Math.PI; // rotation per second

  private final SwerveModuleTalonFX frontLeftModule = new SwerveModuleTalonFX(
      Constants.FRONT_LEFT_DRIVE_TALON_CAN_ID,
      Constants.FRONT_LEFT_TURNING_TALON_CAN_ID,
      Constants.FRONT_LEFT_CANCODER_CAN_ID,
      Constants.FRONT_LEFT_CANCODER_OFFSET);

  private final SwerveModuleTalonFX frontRightModule = new SwerveModuleTalonFX(
      Constants.FRONT_RIGHT_DRIVE_TALON_CAN_ID,
      Constants.FRONT_RIGHT_TURNING_TALON_CAN_ID,
      Constants.FRONT_RIGHT_CANCODER_CAN_ID,
      Constants.FRONT_RIGHT_CANCODER_OFFSET);

  private final SwerveModuleTalonFX rearLeftModule = new SwerveModuleTalonFX(
      Constants.REAR_LEFT_DRIVE_TALON_CAN_ID,
      Constants.REAR_LEFT_TURNING_TALON_CAN_ID,
      Constants.REAR_LEFT_CANCODER_CAN_ID,
      Constants.REAR_LEFT_CANCODER_OFFSET);

  private final SwerveModuleTalonFX rearRightModule = new SwerveModuleTalonFX(
      Constants.REAR_RIGHT_DRIVE_TALON_CAN_ID,
      Constants.REAR_RIGHT_TURNING_TALON_CAN_ID,
      Constants.REAR_RIGHT_CANCODER_CAN_ID,
      Constants.REAR_RIGHT_CANCODER_OFFSET);

  private final WPI_Pigeon2 pidgey = new WPI_Pigeon2(
      Constants.PIDGEON_CAN_ID,
      Constants.CANIVORE_BUS_NAME);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      Constants.FRONT_LEFT_MODULE_LOCATION,
      Constants.FRONT_RIGHT_MODULE_LOCATION,
      Constants.REAR_LEFT_MODULE_LOCATION,
      Constants.REAR_RIGHT_MODULE_LOCATION);

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

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxDriveSpeed);
    for (int i = 0; i < swerveModuleStates.length; i++) {
      SmartDashboard.putString("2. Desaturated SwerveModuleState " + i, swerveModuleStates[i].toString());
    }

    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    rearLeftModule.setDesiredState(swerveModuleStates[2]);
    rearRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        pidgey.getRotation2d(),
        frontLeftModule.getState(),
        frontRightModule.getState(),
        rearLeftModule.getState(),
        rearRightModule.getState());
  }
}