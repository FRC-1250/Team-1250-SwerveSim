package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.PIDGains;

public class Constants {
    public static final String CANIVORE_BUS_NAME = "Party Bus";

    // SwereModule
    public static final int TALONFX_PRIMARY_PID_LOOP_ID = 0;
    public static final int CAN_TIMEOUT_MS = 30;
    public static final double METERS_PER_SECOND_TO_TALON_TICKS_CONVERSION_FACTOR = 5293;
    public static final double DEGRESS_TO_TALON_TICKS_CONVERSION_FACTOR = 4096 / 360;

    // Drivetrain
    /**
     * The disatance between the centers of the right and left wheels on the robot.
     * This value must be in the same unit as {@wheelBase}.
     */

    private static final double TRACK_WIDTH = 0.5;
    /**
     * The distance between the front and back wheels on the robot.
     * This value must be in the same unit as {@trackWidth}.
     */
    private static final double WHEELBASE = 0.5;

    /*
    public static final Translation2d FRONT_LEFT_MODULE_LOCATION = new Translation2d(WHEELBASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d FRONT_RIGHT_MODULE_LOCATION = new Translation2d(WHEELBASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d REAR_LEFT_MODULE_LOCATION = new Translation2d(-WHEELBASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d REAR_RIGHT_MODULE_LOCATION = new Translation2d(-WHEELBASE / 2, TRACK_WIDTH / 2);
    */

    public static final Translation2d FRONT_LEFT_MODULE_LOCATION = new Translation2d(WHEELBASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d FRONT_RIGHT_MODULE_LOCATION = new Translation2d(WHEELBASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d REAR_LEFT_MODULE_LOCATION = new Translation2d(-WHEELBASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d REAR_RIGHT_MODULE_LOCATION = new Translation2d(-WHEELBASE / 2, -TRACK_WIDTH / 2);

    public static final PIDGains DRIVE_TALON_VELOCITY_GAINS = new PIDGains(0.33, 0, 0.3, 1023.0 / 20660.0);
    public static final PIDGains TURNING_TALON_POSITION_GAINS = new PIDGains(0.45, 0.0, 0.15, 0.0);

    public static final int FRONT_LEFT_TURNING_TALON_CAN_ID = 17;
    public static final int FRONT_LEFT_DRIVE_TALON_CAN_ID = 18;
    public static final int FRONT_LEFT_CANCODER_CAN_ID = 19;
    public static final double FRONT_LEFT_CANCODER_OFFSET = -46.14258;

    public static final int FRONT_RIGHT_TURNING_TALON_CAN_ID = 14;
    public static final int FRONT_RIGHT_DRIVE_TALON_CAN_ID = 15;
    public static final int FRONT_RIGHT_CANCODER_CAN_ID = 16;
    public static final double FRONT_RIGHT_CANCODER_OFFSET = -83.14453;

    public static final int REAR_LEFT_TURNING_TALON_CAN_ID = 20;
    public static final int REAR_LEFT_DRIVE_TALON_CAN_ID = 21;
    public static final int REAR_LEFT_CANCODER_CAN_ID = 22;
    public static final double REAR_LEFT_CANCODER_OFFSET = -94.57031;

    public static final int REAR_RIGHT_TURNING_TALON_CAN_ID = 11;
    public static final int REAR_RIGHT_DRIVE_TALON_CAN_ID = 12;
    public static final int REAR_RIGHT_CANCODER_CAN_ID = 13;
    public static final double REAR_RIGHT_CANCODER_OFFSET = 85.78125;

    public static final int PIDGEON_CAN_ID = 23;
}