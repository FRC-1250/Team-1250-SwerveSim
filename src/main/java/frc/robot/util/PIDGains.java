package frc.robot.util;

public class PIDGains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;

	/**
	 * 
	 * @param kP The proportional term produces an output value that is proportional
	 *           to the current error value. The proportional response can be
	 *           adjusted by multiplying the error by a constant Kp, called the
	 *           proportional gain constant.
	 * @param kI The contribution from the integral term is proportional to both the
	 *           magnitude of the error and the duration of the error.
	 * @param kD The derivative of the process error is calculated by determining
	 *           the slope of the error over time and multiplying this rate of
	 *           change by the derivative gain Kd.
	 * @param kF The feedforward term introduces an up front output value to the
	 *           system in order to handle known changes to the system.
	 * 
	 *           https://en.wikipedia.org/wiki/PID_controller#Control_loop_example
	 */

	public PIDGains(double kP, double kI, double kD, double kF) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
	}
}