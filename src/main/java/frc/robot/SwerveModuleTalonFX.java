package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleTalonFX {
    private final WPI_TalonFX driveTalon;
    private final WPI_TalonFX turningTalon;
    private final CANCoder canCoder;

    public SwerveModuleTalonFX(int driveTalonCanID, int turningTalonCanID, int canCoderCanID, double canCoderOffsetDegrees) {
        driveTalon = new WPI_TalonFX(driveTalonCanID, Constants.CANIVORE_BUS_NAME);
        driveTalon.configFactoryDefault();
        driveTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.CAN_TIMEOUT_MS);
        driveTalon.configClosedloopRamp(0.5, Constants.CAN_TIMEOUT_MS);
        driveTalon.config_kF(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DRIVE_TALON_VELOCITY_GAINS.kF, Constants.CAN_TIMEOUT_MS);
        driveTalon.config_kP(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DRIVE_TALON_VELOCITY_GAINS.kP, Constants.CAN_TIMEOUT_MS);
        driveTalon.config_kI(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DRIVE_TALON_VELOCITY_GAINS.kI, Constants.CAN_TIMEOUT_MS);
        driveTalon.config_kD(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DRIVE_TALON_VELOCITY_GAINS.kD, Constants.CAN_TIMEOUT_MS);
        driveTalon.setNeutralMode(NeutralMode.Brake);

        canCoder = new CANCoder(canCoderCanID, Constants.CANIVORE_BUS_NAME);
        canCoder.configFactoryDefault();
        canCoder.configMagnetOffset(canCoderOffsetDegrees, Constants.CAN_TIMEOUT_MS);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, Constants.CAN_TIMEOUT_MS);
        canCoder.configSensorDirection(true, Constants.CAN_TIMEOUT_MS);
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Constants.CAN_TIMEOUT_MS);
        canCoder.setPositionToAbsolute(Constants.CAN_TIMEOUT_MS);

        turningTalon = new WPI_TalonFX(turningTalonCanID, Constants.CANIVORE_BUS_NAME);
        turningTalon.configFactoryDefault();
        turningTalon.configRemoteFeedbackFilter(canCoder, 0, Constants.CAN_TIMEOUT_MS);
        turningTalon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.CAN_TIMEOUT_MS);
        turningTalon.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180, Constants.CAN_TIMEOUT_MS);
        turningTalon.configClosedloopRamp(0.5, Constants.CAN_TIMEOUT_MS);
        turningTalon.config_kF(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TURNING_TALON_POSITION_GAINS.kF, Constants.CAN_TIMEOUT_MS);
        turningTalon.config_kP(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TURNING_TALON_POSITION_GAINS.kP, Constants.CAN_TIMEOUT_MS);
        turningTalon.config_kI(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TURNING_TALON_POSITION_GAINS.kI, Constants.CAN_TIMEOUT_MS);
        turningTalon.config_kD(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TURNING_TALON_POSITION_GAINS.kD, Constants.CAN_TIMEOUT_MS);
        turningTalon.setNeutralMode(NeutralMode.Coast);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveTalon.getSelectedSensorVelocity(), getFromHeading());
      }    

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getFromHeading());
        driveTalon.set(ControlMode.Velocity, state.speedMetersPerSecond * Constants.METERS_PER_SECOND_TO_TALON_TICKS_CONVERSION_FACTOR);
        turningTalon.set(ControlMode.Position, state.angle.getDegrees() * Constants.DEGRESS_TO_TALON_TICKS_CONVERSION_FACTOR);
    }

    private Rotation2d getFromHeading() {
        return Rotation2d.fromDegrees(canCoder.getPosition());
    }
}