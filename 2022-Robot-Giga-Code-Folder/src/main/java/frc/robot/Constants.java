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
            public static final int SERVO_LEFT_PORT = 0;//unassigned
            public static final int SERVO_RIGHT_PORT = 0;//unassigned
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
            //HoodWheels
            public static final double SHOOTER_HOOD_WHEELS_IDLE_VELOCITY = 1000;
            //KickerWheel
            public static final double SHOOTER_KICKER_WHEEL_IDLE_VELOCITY = 1000;
        //Configurations
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
    //IO
        //Controller Ports
        public static final int OPERATOR_CONTROLLER_PORT = 1;
}
