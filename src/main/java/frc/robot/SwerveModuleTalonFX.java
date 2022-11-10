package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    public SwerveModuleTalonFX(int driveTalonCanID, int turningTalonCanID, int canCoderCanID) {
        driveTalon = new WPI_TalonFX(driveTalonCanID);
        driveTalon.configFactoryDefault();
        driveTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.CAN_TIMEOUT_MS);
        driveTalon.configClosedloopRamp(0.5);
        driveTalon.config_kF(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TALONFX_VELOCITY_GAINS.kF, Constants.CAN_TIMEOUT_MS);
        driveTalon.config_kP(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TALONFX_VELOCITY_GAINS.kP, Constants.CAN_TIMEOUT_MS);
        driveTalon.config_kI(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TALONFX_VELOCITY_GAINS.kI, Constants.CAN_TIMEOUT_MS);
        driveTalon.config_kD(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TALONFX_VELOCITY_GAINS.kD, Constants.CAN_TIMEOUT_MS);

        canCoder = new CANCoder(canCoderCanID);
        canCoder.configFactoryDefault();
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        canCoder.configSensorDirection(true);
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        turningTalon = new WPI_TalonFX(turningTalonCanID);
        turningTalon.configFactoryDefault();

        turningTalon.configRemoteFeedbackFilter(canCoder, 0);
        turningTalon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.CAN_TIMEOUT_MS);
        turningTalon.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turningTalon.config_kF(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TALONFX_POSITION_GAINS.kF, Constants.CAN_TIMEOUT_MS);
        turningTalon.config_kP(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TALONFX_POSITION_GAINS.kP, Constants.CAN_TIMEOUT_MS);
        turningTalon.config_kI(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TALONFX_POSITION_GAINS.kI, Constants.CAN_TIMEOUT_MS);
        turningTalon.config_kD(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.TALONFX_POSITION_GAINS.kD, Constants.CAN_TIMEOUT_MS);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveTalon.getSelectedSensorVelocity(), new Rotation2d(turningTalon.getSelectedSensorPosition()));
      }    

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getFromHeading());
        driveTalon.set(ControlMode.Velocity, state.speedMetersPerSecond);
        turningTalon.set(ControlMode.Position, state.angle.getDegrees());
    }

    private Rotation2d getFromHeading() {
        return Rotation2d.fromDegrees(canCoder.getPosition());
    }
    
}
