package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private final XboxController xboxController = new XboxController(0);
    private final JoystickButton rbumper = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);

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
                        false,
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
        return null;
    }
}
