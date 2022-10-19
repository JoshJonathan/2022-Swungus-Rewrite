// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Shooter
        //IDs
            //MainWheel
            public static final int SHOOTER_MAIN_WHEEL_LEFT_ID = 21;
            public static final int SHOOTER_MAIN_WHEEL_RIGHT_ID = 22;
            //HoodWheels
            public static final int SHOOTER_HOOD_WHEELS_ID = 20;
            //KickerWheel
            public static final int SHOOTER_KICKER_WHEEL_ID = 23;
            //Servos
            public static final int SERVO_LEFT_PORT = 0;
            public static final int SERVO_RIGHT_PORT = 9;
        //PID
            //MainWheel
            public static final double SHOOTER_MAIN_WHEEL_KF = 0.0474; //0.0474
            public static final double SHOOTER_MAIN_WHEEL_KP = 0.13; //0.13
            public static final double SHOOTER_MAIN_WHEEL_KI = 0.001; //0.001
            public static final double SHOOTER_MAIN_WHEEL_KI_ZONE = 100.0; //100
            public static final double SHOOTER_MAIN_WHEEL_KD = 0.0;//unassigned
            //HoodWheels
            public static final double SHOOTER_HOOD_WHEELS_KF = 0.051; //0.051
            public static final double SHOOTER_HOOD_WHEELS_KP = 0.1; //0.1
            public static final double SHOOTER_HOOD_WHEELS_KI = 0.001; //0.001
            public static final double SHOOTER_HOOD_WHEELS_KI_ZONE = 250.0; //100
            public static final double SHOOTER_HOOD_WHEELS_KD = 0.0; //unassigned
            //KickerWheel
            public static final double SHOOTER_KICKER_WHEEL_KF = 0.0478; //.0478
            public static final double SHOOTER_KICKER_WHEEL_KP = 0.03; //.03
            public static final double SHOOTER_KICKER_WHEEL_KI = 0.001; //.001
            public static final double SHOOTER_KICKER_WHEEL_KI_ZONE = 100.0; //100
            public static final double SHOOTER_KICKER_WHEEL_KD = 0.0; //0.0
        //Velocities
            //MainWheel
            public static final double SHOOTER_MAIN_WHEEL_IDLE_VELOCITY = 1000; //1000
            public static final double SHOOTER_MAIN_WHEEL_FENDERSHOT_VELOCITY = 6500;//unassigned //6500 for testing
            //HoodWheels
            public static final double SHOOTER_HOOD_WHEELS_IDLE_VELOCITY = 1000; //1000
            public static final double SHOOTER_HOOD_WHEELS_FENDERSHOT_VELOCITY = 10000;//unassigned //10000 for testing
            //KickerWheel
            public static final double SHOOTER_KICKER_WHEEL_IDLE_VELOCITY = 1000; //1000
            public static final double SHOOTER_KICKER_WHEEL_FENDERSHOT_VELOCITY = 4000;//unassigned //4000 for testing
            //servos
            public static final double SHOOTER_SERVOS_VELOCITY = 0.357; //0.357, distance per second, where distance is in servo units and the servo lenth is 2 units
            public static final double SHOOTER_SERVOS_MAXIMUM_TIME_DELTA = 0.06; //0.06 is 3 missed code loops
        //Positions
            //servos
            public static final double SHOOTER_SERVOS_IDLE_POSITION = 0;//unassigned //0 for testing
            public static final double SHOOTER_SERVOS_FENDERSHOT_POSITION = -0.55;//unassigned //-0.5 for testing
            public static final double SHOOTER_HOOD_IDLE_BOTTOM = SHOOTER_SERVOS_FENDERSHOT_POSITION;
            public static final double SHOOTER_HOOD_IDLE_LOW = -0.16;
            public static final double SHOOTER_HOOD_IDLE_HIGH = 0.23;
            public static final double SHOOTER_HOOD_IDLE_TOP = 0.6;
        //Configurations
            //MainWheel
            public static final double SHOOTER_MAIN_WHEEL_RAMP_TIME = 0.25;
            //HoodWheels
            public static final double SHOOTER_HOOD_WHEELS_RAMP_TIME = 0.25;
            //KickerWheel
            public static final double SHOOTER_KICKER_WHEEL_RAMP_TIME = 0.25;
        //Allowable Errors
            //MainWheel
            public static final double SHOOTER_MAIN_WHEEL_ALLOWABLE_ERROR = 100.0; //100/6500
            //HoodWheels
            public static final double SHOOTER_HOOD_WHEELS_ALLOWABLE_ERROR = 100.0; //100/10000
            //KickerWheels
            public static final double SHOOTER_KICKER_WHEEL_ALLOWABLE_ERROR = 100.0; //50/4000
            //Servos
            public static final double SHOOTER_SERVOS_ALLOWABLE_ERROR = 0.02; //0.04/2
////////

    //Drivetrain
        //IDs
            //LeftFront
            public static final int DRIVETRAIN_LEFT_FRONT_ID = 4;
            //LeftRear
            public static final int DRIVETRAIN_LEFT_REAR_ID = 3;
            //RightFront
            public static final int DRIVETRAIN_RIGHT_FRONT_ID = 2;
            //RightRear
            public static final int DRIVETRAIN_RIGHT_REAR_ID = 1;
        //Config Parameters
            //NeutralDeadband
            public static final double DRIVETRAIN_NEUTRAL_DEADBAND = 0.0;//unassigned
            //NominalVoltage
            public static final double DRIVETRAIN_NOMINAL_VOLTAGE = 12.0; //12.5
        //Slew Rates
            //speed
                //regular
                public static final double DRIVETRAIN_SPEED_SLEW_FORWARD = 1.0; //1.2
                public static final double DRIVETRAIN_SPEED_SLEW_REVERSE = 1.4; //1.4
                //max speed
                public static final double DRIVETRAIN_MAX_SPEED_SLEW_FORWARD = 0.6;
                public static final double DRIVETRAIN_MAX_SPEED_SLEW_REVERSE = 1.0;
                //limit speed
                public static final double DRIVETRAIN_LIMIT_OUTPUT_FORWARD = 0.69;
                public static final double DRIVETRAIN_LIMIT_OUTPUT_REVERSE = -0.89;
            //turn
            public static final double DRIVETRAIN_TURN_SLEW = 4.0; //4.0
        //Controller Deadzones
            //speed
            public static final double DRIVETRAIN_SPEED_DEADZONE = 0.00; //0.0
            //turn
            public static final double DRIVETRAIN_TURN_DEADZONE = 0.075; //0.75
        //Minimum Outputs
            //speed
            public static final double DRIVETRAIN_SPEED_MINIMUM_OUTPUT = 0.15; //.15
            //Turn
            public static final double DRIVETRAIN_TURN_MINIMUM_OUTPUT = 0.21; //.21
        //Maximum Outputs
            //Speeds
            public static  final double DRIVETRAIN_MAX_OUTPUT_FORWARD = 1;
            public static  final double DRIVETRAIN_MAX_OUTPUT_REVERSE = 1;
            //Turn
            public static  final double DRIVETRAIN_MAX_TURN_PERCENTAGE = 1.0; //.7
        //Automation Constants
            //Turn kP
            public static final double DRIVETRAIN_TURN_kP = 0.01;//unassigned
            //Allowable error
            public static final double DRIVETRAIN_ALLOWABLE_ERROR = 3;//unassigned
////////

    //Indexer
        //Motor IDs
            //Front
            public static final int INDEXER_FRONT_ID = 32;//unassigned
            //Rear
            public static final int INDEXER_REAR_ID = 31;//unassigned
        //Motor Configs
            //Voltage compensation
            public static final double INDEXER_NOMINAL_VOLTAGE = 12.5;
            public static final int INDEXER_VOLTAGE_FILTER_WINDOW_SAMPLES = 64;
            public static final double INDEXER_RAMP_TIME = .125;
        //Motor Voltage
            //Indexing
            public static final double INDEXER_OUTPUT = 0.35;//unassigned
        //Shooting Constants
            //Timer Delay
            public static final double INDEXER_TIMER_DELAY = 0.25;
    //Intake
        //Solenoid Channels
            //Solenoid
            public static final int INTAKE_SOLENOID_CHANNEL = 0;
        //Motor IDs
            //Motor
            public static final int INTAKE_TALON_ID = 40;
        //Motor Outputs
            //Motor
            public static final double INTAKE_MOTOR_OUTPUT = 1.00;
        //Config Parameters
            //Motor Ramp Time
            public static final double INTAKE_RAMP_TIME = 0.25;
            //Nominal Robot Voltage
            public static final double INTAKE_NOMINAL_ROBOT_VOLTAGE = 12.5;
            //VoltageFilterWindowSamples
            public static final int INTAKE_VOLTAGE_FILTER_WINDOW_SAMPLES = 64;//unassigned
    //Limelight
        //Pipeline
            //Standard
            public static final int LIMELIGHT_STANDARD_PIPELINE = 8;
        //Speed
            //Search
            public static final double LIMELIGHT_SEARCH_SPEED = 0.5;//unassigned
////////
    //Climb
        //Solenoids
        public static final int CLIMB_SOLENOID_CHANNEL = 2;

    //IO
        //Controller Ports
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
////////        

        public static final class DriveTrainConstants{
            public static final double kS = 0.6604; //Volts (static friction) //should really be ~1.8 right? -josh
            public static final double kV = 2.1926; // volts*s/m
            public static final double kA = 0.32167; // volts*s^2/m
            public static final double kP = 3.0265;

            public static final double kTrackwidthMeters = 0.61612;

            public static final double kMaxSpeedMetersPerSecond = 1.83;
            public static final double kMaxAccelerationMetersPerSecondSquared = 2.7;

            public static final double kRamseteB = 2;
            public static final double kRamseteZeta = 0.7;

            public static final double metersToTicks = 25722.88;
        }
}

