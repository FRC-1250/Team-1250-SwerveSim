// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(false);
  }

  @Override
  public void disabledPeriodic() {
  }

  private void driveWithJoystick(boolean fieldRelative) {
    double leftY = m_controller.getLeftY();
    double leftX = m_controller.getLeftX();
    double rightX = m_controller.getRightX();

    SmartDashboard.putString("1. Raw joystick inputs",
        String.format("Joystick inputs(leftY: %s, leftX: %s, rightX: %s)",
            leftY, leftX, rightX));

    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(leftY, 0.1))
        * Drivetrain.maxDriveSpeed * 0.5;

    final var ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(leftX, 0.1))
        * Drivetrain.maxDriveSpeed * 0.5;

    final var rot = m_rotLimiter.calculate(MathUtil.applyDeadband(rightX, 0.1))
        * Drivetrain.maxTurningSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}