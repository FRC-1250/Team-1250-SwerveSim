package frc.robot;

public class Constants {
    public static final int TALONFX_PRIMARY_PID_LOOP_ID = 0;
    public static final int CAN_TIMEOUT_MS = 30;
    public static final PIDGains TALONFX_VELOCITY_GAINS = new PIDGains(0.1, 0.001, 5, 1023.0 / 20660.0);
    public static final PIDGains TALONFX_POSITION_GAINS = new PIDGains(0.15, 0.0, 1.0, 0.0);
}
