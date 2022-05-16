// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalSource;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //IO
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    
    //intake
    public static final double SPEED = 0.82;
    public static final int SOLENOID_CHANNEL = 0;
    public static final int DEVICE_NUMBER = 40;
    public static final double RAMP_TIME = .25;
    public static final double INTAKE_MOTOR_SATURATION_VOLTAGE = 9.5;

    //shooter
    public static final int MAIN_WHEEL_LEFT_DEVICE_NUMBER = 21;
    public static final int MAIN_WHEEL_RIGHT_DEVICE_NUMBER = 22;
    public static final double SHOOTER_RAMP_TIME = .1;
    public static final double SHOOTER_KF = 0.1599;
    public static final double SHOOTER_KP = 0;
    public static final double MAIN_WHEEL_IDLE_VELOCITY = 1000;
    public static final double MAIN_WHEEL_SHOOT_VELOCITY = 8000;
    public static final double SHOOTER_GEARING = 1;
    public static final double SHOOTER_MOI = 0.00221762329;
    public static final double SHOOTER_SIM_LOOP_TIME = 0.02;

    public static final double RPM_TO_FALCON_CONVERSION_CONSTANT = (2.9296875);//600000/204800
}
