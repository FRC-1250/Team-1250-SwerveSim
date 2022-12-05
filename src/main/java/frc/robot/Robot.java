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
    // If swerve module is replaced then calibration has to be done again
    if (m_controller.getStartButton() && m_controller.getBackButton()) {
      m_swerve.frontLeftModule.setPosTalon();
      m_swerve.frontRightModule.setPosTalon();
      m_swerve.backLeftModule.setPosTalon();
      m_swerve.backRightModule.setPosTalon();
      System.out.print("The starting position of the swerve modules has been calibrated");
    }
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.

    double leftY = m_controller.getLeftY();
    double leftX = m_controller.getLeftX();
    double rightX = m_controller.getRightX();

    SmartDashboard.putString("1. Raw joystick inputs",
        String.format("Joystick inputs(leftY: %s, leftX: %s, rightX: %s)",
            leftY, leftX, rightX));

    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(leftY, 0.1))
        * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(leftX, 0.1))
        * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(rightX, 0.1))
        * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);

  }
}
