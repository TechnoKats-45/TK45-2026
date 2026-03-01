package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants 
{
    public class CAN_BUS
    {
        public static final String RIO = "rio";
        public static final String CANIVORE = "canivore";
    }

    public class CAN_ID
    {
        public static final int PIGEON = 2;

        public static final int FRONT_RIGHT_DRIVE = 10;     // Stored in Tuner Constants
        public static final int FRONT_RIGHT_STEER = 11;     // Stored in Tuner Constants
        public static final int FRONT_RIGHT_ENCODER = 12;   // Stored in Tuner Constants

        public static final int BACK_RIGHT_DRIVE = 20;      // Stored in Tuner Constants
        public static final int BACK_RIGHT_STEER = 21;      // Stored in Tuner Constants
        public static final int BACK_RIGHT_ENCODER = 22;    // Stored in Tuner Constants

        public static final int BACK_LEFT_DRIVE = 30;       // Stored in Tuner Constants
        public static final int BACK_LEFT_STEER = 31;       // Stored in Tuner Constants
        public static final int BACK_LEFT_ENCODER = 32;     // Stored in Tuner Constants

        public static final int FRONT_LEFT_DRIVE = 40;      // Stored in Tuner Constants
        public static final int FRONT_LEFT_STEER = 41;      // Stored in Tuner Constants
        public static final int FRONT_LEFT_ENCODER = 42;    // Stored in Tuner Constants

        public static final int SPINDEXER = 50;

        public static final int INTAKE_LEFT = 51;
        public static final int INTAKE_LEFT_ROTATE = 52;
        public static final int INTAKE_RIGHT = 53;
        public static final int INTAKE_RIGHT_ROTATE = 54;

        public static final int SHOOTER_LEFT = 55;
        public static final int SHOOTER_RIGHT = 56;
        public static final int HOOD_ROTATE = 57;
        public static final int TURRET = 58;
        public static final int BALL_ELEVATOR = 59;

        public static final int CLIMBER = 60;
    }

    public class Hood
    {
        public static final double AngleToleranceDegrees = 1.0; // TODO - TUNE

        public static enum HoodPosition 
        {
            TOP(57.065138),     // TODO - TUNE
            MIDDLE(0),          // TODO - TUNE
            BOTTOM(22.315138);

            public final double value;

            private HoodPosition(double value) 
            {
                this.value = value;
            }
        }
    }

    public class Shooter
    {
        public static final Transform3d BALL_TRANSFORM_CENTER = new Transform3d(-0.24, 0, 0.5, Rotation3d.kZero);
        
        public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOT_SPEED = new InterpolatingDoubleTreeMap();
        static {
            DISTANCE_TO_SHOT_SPEED.put(2.07, 7.0);  // TODO - TUNE
            // DISTANCE_TO_SHOT_SPEED.put(2.41, 41.0);
            // DISTANCE_TO_SHOT_SPEED.put(3.20, 45.0);
            // DISTANCE_TO_SHOT_SPEED.put(3.87, 49.0);
            // DISTANCE_TO_SHOT_SPEED.put(4.57, 52.0);
            DISTANCE_TO_SHOT_SPEED.put(4.92, 9.0);  // TODO - TUNE
            // DISTANCE_TO_SHOT_SPEED.put(0.0, 7.0);
            // DISTANCE_TO_SHOT_SPEED.put(5.0, 8.25);
            // DISTANCE_TO_SHOT_SPEED.put(10.0, 10.0);
        }
    }


    public class Turret
    {     
        public static final double AngleToleranceDegrees = 1.0; // TODO - TUNE

        public static enum TurretPosition 
        {
            LEFT(80.419921875),
            CENTER(0),   
            RIGHT(-86.484375);    

            public final double value;

            private TurretPosition(double value) 
            {
                this.value = value;
            }
        }
    }

    public class Ball_Elevator
    {
        public static final double ELEVATOR_SPEED = 0.5; // TODO - TUNE
        public static final double MAX_ELEVATOR_SPEED_RPS = 129; // TODO - TUNE
    }

    public class LeftIntake
    {
        public static final double INTAKE_SPEED = 0.5; // TODO - TUNE
        public static final double INTAKE_ROTATE_SPEED = 0.5; // TODO - TUNE
        public static final double ANGLE_TOLERANCE_DEGREES = 1; // TODO - TUNE
        public static final double MAX_PIVOT_ANGLE = 0.5 ;// TODO - tune
        public static final double MIN_PIVOT_ANGLE = 0 ;// TODO -tune
        public static final double SPEED_TOLERANCE_RPS = 0.5; // TODO - TUNE
    }
    public class RightIntake
    {
        public static final double INTAKE_SPEED = 0.5; // TODO - TUNE
        public static final double INTAKE_ROTATE_SPEED = 0.5; // TODO - TUNE
        public static final double ANGLE_TOLERANCE_DEGREES = 1; // TODO - TUNE
        public static final double MAX_PIVOT_ANGLE = 0.5 ;// TODO - tune
        public static final double MIN_PIVOT_ANGLE = 0 ;// TODO -tune
        public static final double SPEED_TOLERANCE_RPS = 0.5; // TODO - TUNE
    }

    public class Climber
    {
        public static final double CLIMB_SPEED = 0.5; // TODO - TUNE
        public static final double HEIGHT_TOLERANCE_INCHES = 0.5; // TODO - TUNE
        public static final double ROTATIONS_PER_INCH = 1.0; // TODO - set mechanism conversion
        public static final double MIN_HEIGHT_INCHES = 21.0; // TODO - tune
        public static final double MAX_HEIGHT_INCHES = 29.0; // TODO - tune
    }

    public class Spindexer
    {
        public static final double MAX_SPINDEX_SPEED_RPS = 101.0; // TODO - TUNE
        public static final double SPINDEXER_SPEED = 0.5; // TODO - TUNE
    }
}
