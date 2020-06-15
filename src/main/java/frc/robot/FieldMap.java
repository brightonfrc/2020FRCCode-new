package frc.robot;

public class FieldMap {
    /**
    * This class contains the dimensions of the feild
    */

    // dimensions
    public static final int X = 0;
    public static final int Y = 1;
    public static final int Z = 1;

    // note that all the measurements are in cm

    /*********************************************
     * FIELD
     *********************************************/

    public static final double[] FIELD_DIMENSIONS = { 821.055, 1598.295 };

    /*********************************************
     * SHIELD GENERATOR(SEE-SAW)
     *********************************************/

    public static final double SHIELD_GENERATOR_BAR_HEIGHT = 160.02;
    public static final double[] SHIELD_GENERATOR_DIMENSIONS = { 430.53, 459.105, 290.83 };

    // Note: there is a stopper in the centre, so the robot can not hang there
    public static final double BAR_LENGTH_FROM_END_TO_END = 290.195;
    public static final double BAR_LENGTH_FROM_EDGE_TO_CENTRE = 141.2875;

    /*********************************************
     * TRENCH(WHEEL OF FORTUNE)
     *********************************************/

    public static final double LOWEST_WHEEL_HEIGHT = 76.835;
    public static final double WHEEL_DIAMETER = 81.28;

    public static final int MIN_WHEEL_ROTATIONS = 3;
    public static final int MAX_WHEEL_ROTATIONS = 5;

    /*********************************************
     * SHOOTING
     *********************************************/

    public static final double BALL_DIAMETER = 17.78;

    // outer port target dimensions (cm)
    // 3ft 3 1/4 in by 1 ft. 5 in
    public final static double[] OUTER_PORT_TARGET_DIMS = { 99.695, 43.18 };
    public final static double TARGET_HEIGHT = 2.49555; // Metres

    public final static double DISTANCE_OF_INNER_PORT_FROM_OUTER_PORT = 74.295;
    public final static double DIAMETER_OF_THE_OUTER_PORT = 76.2; // TODO: try to shoot slightly higher because if the ball hits the wall below the inner port, it will bounce outside of the outer port. If it hits the wall above the inner port, it will still count for the upper port.

    // loading bay target dimensions
    // 7 in by 11 in
    // 11 in from the ground
    public final static double[] LOADING_BAY_TARGET_DIMS = { 17.78, 27.94 };
}