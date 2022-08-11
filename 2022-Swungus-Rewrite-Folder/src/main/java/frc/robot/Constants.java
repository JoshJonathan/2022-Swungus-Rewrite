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
            public static final double SHOOTER_MAIN_WHEEL_KF = .0532;
            public static final double SHOOTER_MAIN_WHEEL_KP = 0.2; //dont really know
            //HoodWheels
            public static final double SHOOTER_HOOD_WHEELS_KF = 0.058; 
            public static final double SHOOTER_HOOD_WHEELS_KP = 0.02; //dont really know
            //KickerWheel
            public static final double SHOOTER_KICKER_WHEEL_KF = 0.046;
            public static final double SHOOTER_KICKER_WHEEL_KP = 0.0; //dont really know
        //Velocities
            //MainWheel
            public static final double SHOOTER_MAIN_WHEEL_IDLE_VELOCITY = 1000;
            public static final double SHOOTER_MAIN_WHEEL_FENDERSHOT_VELOCITY = 5000;//unassigned
            //HoodWheels
            public static final double SHOOTER_HOOD_WHEELS_IDLE_VELOCITY = 1000;
            public static final double SHOOTER_HOOD_WHEELS_FENDERSHOT_VELOCITY = 8000;//unassigned
            //KickerWheel
            public static final double SHOOTER_KICKER_WHEEL_IDLE_VELOCITY = 1000;
            public static final double SHOOTER_KICKER_WHEEL_FENDERSHOT_VELOCITY = 3000;//unassigned
        //Positions
            //servos
            public static final double SHOOTER_SERVOS_IDLE_POSITION = 0;//unassigned
            public static final double SHOOTER_SERVOS_FENDERSHOT_POSITION = 0;//unassigned
        //Configurations
            //Velocity Measurement Window
            public static final int SHOOTER_VELOCITY_MEASUREMENT_WINDOW = 32;
            //MainWheel
            public static final double SHOOTER_MAIN_WHEEL_RAMP_TIME = 0.25;
            //HoodWheels
            public static final double SHOOTER_HOOD_WHEELS_RAMP_TIME = 0.25;
            //KickerWheel
            public static final double SHOOTER_KICKER_WHEEL_RAMP_TIME = 0.25;
        //Allowable Errors
            //MainWheel
            public static final double SHOOTER_MAIN_WHEEL_ALLOWABLE_ERROR = 0.00625; //50/8000
            //HoodWheels
            public static final double SHOOTER_HOOD_WHEELS_ALLOWABLE_ERROR = 0.005; //50/10000
            //KickerWheels
            public static final double SHOOTER_KICKER_WHEEL_ALLOWABLE_ERROR = 0; //unassigned
            //Servos
            public static final double SHOOTER_SERVOS_ALLOWABLE_ERROR = 0;//unassigned
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
            public static final double DRIVETRAIN_NEUTRAL_DEADBAND = 0.0;//unassigned, .1
            //NominalVoltage
            public static final double DRIVETRAIN_NOMINAL_VOLTAGE = 12.5;//unsure
            //VoltageFilterWindowSamples
            public static final int DRIVETRAIN_VOLTAGE_FILTER_WINDOW_SAMPLES = 64;//unsure
        //Slew Rates
            //speed
            public static final double DRIVETRAIN_SPEED_SLEW = 0.8;
            //turn
            public static final double DRIVETRAIN_TURN_SLEW = 2.0;
        //Controller Deadzones
            //speed
            public static final double DRIVETRAIN_SPEED_DEADZONE = 0.00;
            //turn
            public static final double DRIVETRAIN_TURN_DEADZONE = 0.075;
        //Minimum Outputs
            //speed
            public static final double DRIVETRAIN_SPEED_MINIMUM_OUTPUT = 0.11;
            //Turn
            public static final double DRIVETRAIN_TURN_MINIMUM_OUTPUT = 0.19;
        //Maximum Outputs
            //Turn
            public static double DRIVETRAIN_MAX_TURN_PERCENTAGE = .6;
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
            public static final double INDEXER_RAMP_TIME = .05;
        //Motor Voltage
            //Indexing
            public static final double INDEXER_OUTPUT = 0.5;//unassigned
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

    //Elevator
        //Motor IDs
            //Motor
            public static final int ELEVATOR_MOTOR_ID = 05; //unassigned
        //Limit Switch
            //Bottom
            public static final int ELEVATOR_LIMIT_SWITCH_DIO = 0;//unassigned

        //Outputs
            //Elevator
            public static final double ELEVATOR_SPEED = 0.3;



////////
    //IO
        //Controller Ports
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
}
