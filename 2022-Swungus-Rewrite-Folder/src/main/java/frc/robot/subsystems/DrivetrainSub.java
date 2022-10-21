// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.JoshSlewFilter;

public class DrivetrainSub extends SubsystemBase {
  //Motor Controllers
    //left
    WPI_TalonFX drivetrainLeftFront = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_FRONT_ID);
    WPI_TalonFX drivetrainLeftRear = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_REAR_ID);
    //right
    WPI_TalonFX drivetrainRightFront = new WPI_TalonFX(Constants.DRIVETRAIN_RIGHT_FRONT_ID);
    WPI_TalonFX drivetrainRightRear = new WPI_TalonFX(Constants.DRIVETRAIN_RIGHT_REAR_ID);
  //Gyro
    public final AHRS m_gyro = new AHRS(SPI.Port.kMXP); // maybe replace type with AHRS
  //Drivetrain
    DifferentialDrive arcadeDrive = new DifferentialDrive(drivetrainLeftFront, drivetrainRightFront);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.DriveTrainConstants.kTrackwidthMeters);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d().times(-1)); //should be set after gyro initializes to get an accurate value; -josh
  //Input Filters
    //SlewRateLimiter speedFilter = new SlewRateLimiter(Constants.DRIVETRAIN_SPEED_SLEW_FORWARD);
    //SlewRateLimiter speedFilterReverse = new SlewRateLimiter(Constants.DRIVETRAIN_SPEED_SLEW_REVERSE);
    SlewRateLimiter turnFilter = new SlewRateLimiter(Constants.DRIVETRAIN_TURN_SLEW);
    JoshSlewFilter speedFilter = new JoshSlewFilter(Constants.DRIVETRAIN_SPEED_SLEW_FORWARD, Constants.DRIVETRAIN_SPEED_SLEW_REVERSE, 0, Constants.DRIVETRAIN_MAX_SPEED_SLEW_FORWARD, Constants.DRIVETRAIN_MAX_SPEED_SLEW_REVERSE, Constants.DRIVETRAIN_MAX_OUTPUT_FORWARD, Constants.DRIVETRAIN_MAX_OUTPUT_REVERSE, Constants.DRIVETRAIN_LIMIT_OUTPUT_FORWARD, Constants.DRIVETRAIN_LIMIT_OUTPUT_REVERSE);
  //Values
  double dt_lt;
  double dt_rt;
  double dt_lx;
  double dt_speed;
  double dt_turn;
  //Static Variables	
    //last turn	
    static boolean lastTurnRight;	
    
  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    turnControlTimer.start();
    //Gyro Calibration
      m_gyro.calibrate();
    //Motor Controller Configs
      //Left
        //Front
        drivetrainLeftFront.configFactoryDefault();
        drivetrainLeftFront.configNeutralDeadband(Constants.DRIVETRAIN_NEUTRAL_DEADBAND);
        drivetrainLeftFront.configVoltageCompSaturation(12.0);
        drivetrainLeftFront.enableVoltageCompensation(true);
        drivetrainLeftFront.setInverted(TalonFXInvertType.Clockwise);
        drivetrainLeftFront.setNeutralMode(NeutralMode.Coast);
        //Rear
        drivetrainLeftRear.follow(drivetrainLeftFront);
        drivetrainLeftRear.setInverted(TalonFXInvertType.FollowMaster);
        drivetrainLeftRear.setNeutralMode(NeutralMode.Coast);
        drivetrainLeftRear.configVoltageCompSaturation(12.0);
        drivetrainLeftRear.enableVoltageCompensation(true);
      //Right
        //Front
        drivetrainRightFront.configFactoryDefault();
        drivetrainRightFront.configNeutralDeadband(Constants.DRIVETRAIN_NEUTRAL_DEADBAND);
        drivetrainRightFront.configVoltageCompSaturation(12.0);
        drivetrainRightFront.enableVoltageCompensation(true);
        drivetrainRightFront.setInverted(TalonFXInvertType.CounterClockwise);
        drivetrainRightFront.setNeutralMode(NeutralMode.Coast);
        //Rear
        drivetrainRightRear.follow(drivetrainRightFront);
        drivetrainRightRear.setInverted(TalonFXInvertType.FollowMaster);
        drivetrainRightRear.setNeutralMode(NeutralMode.Coast);
        drivetrainRightRear.configVoltageCompSaturation(12.0);
        drivetrainRightRear.enableVoltageCompensation(true);
        //All
        resetEncoders();
        enableCurrentLimiting(true);
    //Drivetrain Configs
      arcadeDrive.setDeadband(0);
    //Static Variables
      //last turn
      lastTurnRight = true;
  }

  public void enableVoltageCompensation(boolean onOff) {
      drivetrainRightRear.enableVoltageCompensation(onOff);
      drivetrainLeftRear.enableVoltageCompensation(onOff);
      drivetrainRightFront.enableVoltageCompensation(onOff);
      drivetrainLeftFront.enableVoltageCompensation(onOff);
  }

  SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 55, 0, 0);
  SupplyCurrentLimitConfiguration currentLimitOff = new SupplyCurrentLimitConfiguration(false, 55, 0, 0);
  public void enableCurrentLimiting(boolean onOff) {
    if (onOff) {
      drivetrainRightRear.configSupplyCurrentLimit(currentLimit);
      drivetrainLeftRear.configSupplyCurrentLimit(currentLimit);
      drivetrainRightFront.configSupplyCurrentLimit(currentLimit);
      drivetrainLeftFront.configSupplyCurrentLimit(currentLimit);  
    }
    else {
      drivetrainRightRear.configSupplyCurrentLimit(currentLimitOff);
      drivetrainLeftRear.configSupplyCurrentLimit(currentLimitOff);
      drivetrainRightFront.configSupplyCurrentLimit(currentLimitOff);
      drivetrainLeftFront.configSupplyCurrentLimit(currentLimitOff);
    }
}

  //Drive
  public void drive(double irt, double ilt, double ilx) {
    deadzoneInputs(irt, ilt, ilx);
    simplifyInputs();
    filterValues();
    scaleValues();
    arcadeDrive(dt_speed, dt_turn);
  }

  //Deadzone Inputs
  public void deadzoneInputs(double irt, double ilt, double ilx) {
    if(irt < Constants.DRIVETRAIN_SPEED_DEADZONE) {
      dt_rt=0;
    } else dt_rt=irt;

    if(ilt < Constants.DRIVETRAIN_SPEED_DEADZONE) {
      dt_lt=0;
    } else dt_lt=ilt;

    if(Math.abs(ilx) < Constants.DRIVETRAIN_TURN_DEADZONE) {
      dt_lx=0;
    } else dt_lx=ilx;
  }

  //Simplify Inputs
  public void simplifyInputs() {
    dt_speed = dt_rt-dt_lt;
    dt_turn = dt_lx;
  }

  //filter Inputs
  public void filterValues() {
    //speed
    dt_speed = speedFilter.calculate(dt_speed);
    //turn
    dt_turn = turnFilter.calculate(dt_turn);
  }

  
  //Scale Values
    /*
      These equations modify the speed and turn inputs by adding a constant which serves to decrease looseness in the sticks,
      and by linearly scaling the new inputs such that the max output on the controller still cooresponds to max output on the motor,
      rather than max output plus the constant that was added before.
    */
  public void scaleValues() {
    //Speed
    if (dt_speed > 0) {
      dt_speed = (dt_speed)+(Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)-((Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)*(dt_speed));
    }
    if (dt_speed < 0) {
      dt_speed = (dt_speed)-(Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)+((Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)*(-dt_speed));
    }
    //Turn
    if (dt_turn > 0) {
      dt_turn = (Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)+(dt_turn)-((Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)*(dt_turn))-((1-Constants.DRIVETRAIN_MAX_TURN_PERCENTAGE)*(dt_turn));
      lastTurnRight = true;
    }
    if (dt_turn < 0) {
      dt_turn = (-Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)+(dt_turn)-((Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)*(dt_turn))-((1-Constants.DRIVETRAIN_MAX_TURN_PERCENTAGE)*(dt_turn));
      lastTurnRight = false;
    }
  }

  //Arcade Drive
  public void arcadeDrive(double speed, double turn) {
    arcadeDrive.arcadeDrive(speed, turn);
    //SmartDashboard.putNumber("voltageOut", speed);
  }
  static Timer turnControlTimer = new Timer();
  static double lastControlTime = 0;
  
  public void arcadeDriveControlledTurn(double speed, double turn) {
  //  if(turnControlTimer.get()-lastControlTime>1){ turnControlTimer.reset(); turnControlTimer.start();}
    double rotationToAdd = turn*(turnControlTimer.get()-lastControlTime)*Constants.TELEOP_DEGREES_PER_SECOND;
    desiredAngle+=rotationToAdd;
    //arcadeDrive(speed, desiredTurn());
    arcadeDrive(speed,desiredTurn()/12.0);
    lastControlTime = turnControlTimer.get();

    //dashboard entries
    SmartDashboard.putNumber("rotationToAdd", rotationToAdd);
    SmartDashboard.putNumber("time", turnControlTimer.get());
    SmartDashboard.putNumber("desiredTurn", desiredTurn()/12.0);
    SmartDashboard.putNumber("desiredAngle", desiredAngle);
    SmartDashboard.putNumber("turn", turn);
    SmartDashboard.putNumber("timeDiff", (turnControlTimer.get()-lastControlTime));
  }

  public double desiredTurn(){
   return Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT+Constants.DRIVETRAIN_TURN_TELE_kP*getTurnAngleDifference();
  }

  public double getTurnAngleDifference(){
    return desiredAngle - m_gyro.getRotation2d().times(-1).getDegrees(); //this might need reversed
  }

  public double desiredAngle = 180;

  

  //Aim
  public void aim() {
    arcadeDrive(0, LimelightSub.turn);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(drivetrainLeftFront.getSelectedSensorVelocity()/Constants.DriveTrainConstants.metersToTicks, drivetrainRightFront.getSelectedSensorVelocity()/Constants.DriveTrainConstants.metersToTicks);
  }
  
  public void resetOdometry() {
    resetEncoders();
    desiredAngle = m_gyro.getRotation2d().times(-1).getDegrees();
    System.out.println(">:(");
    m_odometry.resetPosition(new Pose2d(), m_gyro.getRotation2d().times(-1));
  }


  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    System.out.println(">:(");
    desiredAngle = m_gyro.getRotation2d().times(-1).getDegrees();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d().times(-1));
  }

  public void resetEncoders(){
    drivetrainLeftFront.setSelectedSensorPosition(0);
    drivetrainRightFront.setSelectedSensorPosition(0); 
  }
  public double getAverageEncoderDistance() {
    return (drivetrainLeftFront.getSelectedSensorPosition()*Constants.DriveTrainConstants.metersToTicks + drivetrainRightFront.getSelectedSensorPosition()*Constants.DriveTrainConstants.metersToTicks) / 2.0;
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public void tankDriveVolts(double l, double r){
      drivetrainLeftFront.setVoltage(l);
      drivetrainRightFront.setVoltage(r);
      arcadeDrive.feed();
  }



  public double getHeading() {
    return -m_gyro.getRotation2d().getDegrees();
  }
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("xPosition", getPose().getX());
    SmartDashboard.putNumber("yPosition", getPose().getY());

    m_odometry.update(
      m_gyro.getRotation2d().times(-1), drivetrainLeftFront.getSelectedSensorPosition()/Constants.DriveTrainConstants.metersToTicks, drivetrainRightFront.getSelectedSensorPosition()/Constants.DriveTrainConstants.metersToTicks);
  }
}