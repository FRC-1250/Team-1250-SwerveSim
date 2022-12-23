// Copyright (c) FIRST and other WPILib contriutors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveSwerve extends CommandBase {

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  private final Drivetrain drivetrain;
  private final BooleanSupplier boostInputSupplier;
  private final Double throttle;
  private final DoubleSupplier yInputSupplier;
  private final DoubleSupplier xInputSupplier;
  private final DoubleSupplier rotationInputSupplier;
  private final boolean fieldRelative;
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  public DriveSwerve(BooleanSupplier boostInputSupplier, Double throttle, DoubleSupplier yInputSupplier,
      DoubleSupplier xInputSupplier,
      DoubleSupplier rotationInputSupplier, boolean fieldRelative, Drivetrain drivetrain) {
    this.boostInputSupplier = boostInputSupplier;
    this.throttle = throttle;
    this.yInputSupplier = yInputSupplier;
    this.xInputSupplier = xInputSupplier;
    this.rotationInputSupplier = rotationInputSupplier;
    this.drivetrain = drivetrain;
    this.fieldRelative = fieldRelative;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    xSpeed = -yInputSupplier.getAsDouble();
    ySpeed = -xInputSupplier.getAsDouble();
    rotSpeed = -rotationInputSupplier.getAsDouble();

    if (!boostInputSupplier.getAsBoolean()) {
      ySpeed = ySpeed * throttle;
      xSpeed = xSpeed * throttle;
    }

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);
    rotSpeed = MathUtil.applyDeadband(rotSpeed, 0.1);

    xSpeed = m_xspeedLimiter.calculate(xSpeed);
    ySpeed = m_yspeedLimiter.calculate(ySpeed);
    rotSpeed = m_rotLimiter.calculate(rotSpeed);

    xSpeed = xSpeed * Drivetrain.maxDriveSpeed;
    ySpeed = ySpeed * Drivetrain.maxDriveSpeed;
    rotSpeed = rotSpeed * Drivetrain.maxTurningSpeed;

    drivetrain.drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }
}
