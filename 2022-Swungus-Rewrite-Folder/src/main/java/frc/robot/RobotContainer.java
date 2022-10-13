// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.ShooterSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.opencv.video.TrackerGOTURN;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //IO
    //Controllers
    XboxController rc_driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
    XboxController rc_operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
  //Shooter
    private final ShooterSub rc_shootersub = new ShooterSub();
    //commands
      //default command
      private final Command rc_idleshooter = new RunCommand(()-> rc_shootersub.outputToShooter(Constants.SHOOTER_MAIN_WHEEL_IDLE_VELOCITY,
                                                                                               Constants.SHOOTER_HOOD_WHEELS_IDLE_VELOCITY,
                                                                                               Constants.SHOOTER_KICKER_WHEEL_IDLE_VELOCITY,
                                                                                               Constants.SHOOTER_SERVOS_IDLE_POSITION), rc_shootersub);
      //Fendershot
      private final Command rc_fendershot = new RunCommand(()-> rc_shootersub.outputToShooter(Constants.SHOOTER_MAIN_WHEEL_FENDERSHOT_VELOCITY,
                                                                                              Constants.SHOOTER_HOOD_WHEELS_FENDERSHOT_VELOCITY,
                                                                                              Constants.SHOOTER_KICKER_WHEEL_FENDERSHOT_VELOCITY,
                                                                                              Constants.SHOOTER_SERVOS_FENDERSHOT_POSITION), rc_shootersub);
      //PID tuning
      //private final Command rc_setconstants = new InstantCommand(rc_shootersub::setConstants, rc_shootersub);
  //Drivetrain
    private final DrivetrainSub rc_drivetrainsub = new DrivetrainSub();
      //default command
      private final Command rc_drive = new RunCommand(()-> rc_drivetrainsub.drive(rc_driverController.getRightTriggerAxis(),
                                                                                  rc_driverController.getLeftTriggerAxis(),
                                                                                  rc_driverController.getLeftX()), rc_drivetrainsub);
  //Indexer
    private final IndexerSub rc_indexersub = new IndexerSub();
      //Index
      private final Command rc_indexup = new RunCommand(()-> rc_indexersub.index(Constants.INDEXER_OUTPUT), rc_indexersub);
      private final Command rc_indexdown = new RunCommand(()-> rc_indexersub.index(-Constants.INDEXER_OUTPUT), rc_indexersub);
      private final Command rc_indexstop = new RunCommand(()-> rc_indexersub.index(0), rc_indexersub);
      //Shoot
      private final Command rc_indexshoot = new RunCommand(rc_indexersub::shoot, rc_indexersub);
  //Intake
    private final IntakeSub rc_intakesub = new IntakeSub();
    //Commands
      //Deploy
      private final Command rc_deployIntake = new InstantCommand(rc_intakesub::deployIntake, rc_intakesub);
      //Retract
      private final Command rc_retractIntake = new InstantCommand(rc_intakesub::retractIntake, rc_intakesub);
  //limeLight
    private final LimelightSub rc_limelightsub = new LimelightSub();
      //LimelightShot
      private final Command rc_limelightShotDrive = new RunCommand(rc_drivetrainsub::aim, rc_drivetrainsub);
      private final Command rc_limelightShotSpinShooter = new RunCommand(rc_shootersub::limelightShot, rc_shootersub);
  //Climb
    private final ClimbSub rc_climbsub = new ClimbSub();
      //Extend
      private final Command rc_climbextend = new InstantCommand(rc_climbsub::extendClimb, rc_climbsub);
      //Retract
      private final Command rc_climbretract = new InstantCommand(rc_climbsub::retractClimb, rc_climbsub);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //set default commands
    rc_shootersub.setDefaultCommand(rc_idleshooter);
    rc_drivetrainsub.setDefaultCommand(rc_drive);


    m_chooser.setDefaultOption("Two ball - hangar-facing", twoBallAutoHangar());
    m_chooser.addOption("Two ball - human player-facing", twoBallAutoHumanPlayer());

// Put the chooser on the dashboard
SmartDashboard.putData(m_chooser);
  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Shooter
      //Fendershot
      new JoystickButton(rc_operatorController, XboxController.Button.kB.value).whileHeld(rc_fendershot);
      //PID Tuning
      //new JoystickButton(rc_operatorController, XboxController.Button.kA.value).whenPressed(rc_setconstants);
    //Indexer
      //Index
      new JoystickButton(rc_operatorController, XboxController.Button.kRightBumper.value).whileHeld(rc_indexup).whenReleased(rc_indexstop);
      new JoystickButton(rc_operatorController, XboxController.Button.kLeftBumper.value).whileHeld(rc_indexdown).whenReleased(rc_indexstop);
      //Shoot
      new JoystickButton(rc_driverController, XboxController.Button.kA.value).whileHeld(rc_indexshoot).whenReleased(rc_indexstop);
    //Intake
      //deploy
      new JoystickButton(rc_driverController, XboxController.Button.kRightBumper.value).whenPressed(rc_deployIntake);
      new JoystickButton(rc_driverController, XboxController.Button.kLeftBumper.value).whenPressed(rc_retractIntake);
    //Limelight
      new JoystickButton(rc_driverController, XboxController.Button.kX.value).whileHeld(rc_limelightShotDrive);
      new JoystickButton(rc_driverController, XboxController.Button.kX.value).whileHeld(rc_limelightShotSpinShooter);
    //Climb
      new JoystickButton(rc_driverController, XboxController.Button.kStart.value).whenPressed(rc_climbextend);
      new JoystickButton(rc_driverController, XboxController.Button.kBack.value).whenPressed(rc_climbretract);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }



  public Command twoBallAutoHangar(){

    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(/*new Translation2d(3.4, -.75) ,new Translation2d(2, -1)*/),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-1, -1.6, new Rotation2d(Math.PI/2)),
            // Pass config
            swungusTrajectoryConfig(true)
            );

    // Reset odometry to the starting pose of the trajectory.
    rc_drivetrainsub.resetOdometry(trajectory.getInitialPose());
    rc_drivetrainsub.enableVoltageCompensation(false);

    Command oneBall = new ParallelCommandGroup(rc_fendershot, rc_indexshoot).until(()-> Robot.getTime()>5);
    Command stopOneBall = new ParallelCommandGroup(rc_idleshooter, rc_indexstop).until(()->true);
    Command deployIntake = new ParallelCommandGroup(rc_deployIntake).until(()->true);
    CommandGroupBase.clearGroupedCommands();
    Command twoBall = new ParallelCommandGroup(rc_limelightShotDrive, rc_limelightShotSpinShooter, rc_indexshoot).until(()->Robot.getTime()>14);
    CommandGroupBase.clearGroupedCommands();
    Command stopTwoBall = new ParallelCommandGroup(rc_idleshooter, rc_drive, rc_indexstop, rc_retractIntake).until(()->true);
    CommandGroupBase.clearGroupedCommands();
    Command twoBallAuto = new SequentialCommandGroup(/*oneBall, stopOneBall,*/ deployIntake, swungusRamseteCommand(trajectory), twoBall, stopTwoBall);
    CommandGroupBase.clearGroupedCommands();
    
    return twoBallAuto.andThen(() -> rc_drivetrainsub.tankDriveVolts(0, 0)).andThen(() -> rc_drivetrainsub.enableVoltageCompensation(true));

  }

  public Command twoBallAutoHumanPlayer(){

    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(/*new Translation2d(3.4, -.75) ,new Translation2d(2, -1)*/),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-1, 1.6, new Rotation2d(-Math.PI/2)),
            // Pass config
            swungusTrajectoryConfig(true)
            );

    // Reset odometry to the starting pose of the trajectory.
    rc_drivetrainsub.resetOdometry(trajectory.getInitialPose());
    rc_drivetrainsub.enableVoltageCompensation(false);

    Command oneBall = new ParallelCommandGroup(rc_fendershot, rc_indexshoot).until(()-> Robot.getTime()>5);
    Command stopOneBall = new ParallelCommandGroup(rc_idleshooter, rc_indexstop).until(()->true);
    Command deployIntake = new ParallelCommandGroup(rc_deployIntake).until(()->true);
    CommandGroupBase.clearGroupedCommands();
    Command twoBall = new ParallelCommandGroup(rc_limelightShotDrive, rc_limelightShotSpinShooter, rc_indexshoot).until(()->Robot.getTime()>14);
    CommandGroupBase.clearGroupedCommands();
    Command stopTwoBall = new ParallelCommandGroup(rc_idleshooter, rc_drive, rc_indexstop, rc_retractIntake).until(()->true);
    CommandGroupBase.clearGroupedCommands();
    Command twoBallAuto = new SequentialCommandGroup(/*oneBall, stopOneBall,*/ deployIntake, swungusRamseteCommand(trajectory), twoBall, stopTwoBall);
    CommandGroupBase.clearGroupedCommands();
    
    return twoBallAuto.andThen(() -> rc_drivetrainsub.tankDriveVolts(0, 0)).andThen(() -> rc_drivetrainsub.enableVoltageCompensation(true));

  }

  public RamseteCommand swungusRamseteCommand(Trajectory trajectory){
    return new RamseteCommand(
      trajectory,
      rc_drivetrainsub::getPose,
      new RamseteController(Constants.DriveTrainConstants.kRamseteB, Constants.DriveTrainConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
        Constants.DriveTrainConstants.kS,
        Constants.DriveTrainConstants.kV,
        Constants.DriveTrainConstants.kA),
      DrivetrainSub.kDriveKinematics,
      rc_drivetrainsub::getWheelSpeeds,
      new PIDController(Constants.DriveTrainConstants.kP, 0, 0),
      new PIDController(Constants.DriveTrainConstants.kP, 0, 0),
      // RamseteCommand passes volts to the callback
      rc_drivetrainsub::tankDriveVolts,
      rc_drivetrainsub);
  }

  public TrajectoryConfig swungusTrajectoryConfig(boolean setReversed){
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.DriveTrainConstants.kS, Constants.DriveTrainConstants.kV, Constants.DriveTrainConstants.kA), DrivetrainSub.kDriveKinematics, 10);

    return new TrajectoryConfig(
        Constants.DriveTrainConstants.kMaxSpeedMetersPerSecond,
        Constants.DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DrivetrainSub.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        .setReversed(setReversed);
  }

}