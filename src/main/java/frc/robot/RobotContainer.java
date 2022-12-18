package frc.robot;

import java.util.List;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
	private final XboxController xboxController = new XboxController(0);
	private final JoystickButton rbumper = new JoystickButton(xboxController,
			XboxController.Button.kRightBumper.value);

	private final Drivetrain m_swerve = new Drivetrain();

	public RobotContainer() {
		configureButtonBindings();
		m_swerve.setDefaultCommand(
				new DriveSwerve(
						rbumper::get,
						0.75,
						xboxController::getLeftY,
						xboxController::getLeftX,
						xboxController::getRightX,
						true,
						m_swerve));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
				Drivetrain.maxDriveSpeed,
				Drivetrain.maxDriveAcceleration);

		// Apply the constraints for the swerve module kinematics
		trajectoryConfig.setKinematics(m_swerve.kinematics);

		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
				List.of(
						new Pose2d(0, 0, new Rotation2d(0)),
						new Pose2d(1, 0, new Rotation2d(0)),
						new Pose2d(1, 1, new Rotation2d(-90)),
						new Pose2d(2, 1, new Rotation2d(0)),
						new Pose2d(2, 0, new Rotation2d(90))),
				trajectoryConfig);

		ProfiledPIDController thetaController = new ProfiledPIDController(
				AutoConstants.kPThetaController,
				0,
				0,
				AutoConstants.kThetaControllerConstraints);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
				trajectory,
				m_swerve::getPose, // Functional interface to feed supplier
				m_swerve.kinematics,

				// Position controllers
				new PIDController(AutoConstants.kPXController, 0, 0),
				new PIDController(AutoConstants.kPYController, 0, 0),
				thetaController,
				m_swerve::setModuleStates,
				m_swerve);

		// Reset odometry to the starting pose of the trajectory.
		m_swerve.resetOdometry(trajectory.getInitialPose());

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> m_swerve.drive(0, 0, 0, false));
	}
}
