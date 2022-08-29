// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.ShooterSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


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
      //Deploy
      private final Command rc_deployIntake = new InstantCommand(rc_intakesub::deployIntake, rc_intakesub);
      //Retract
      private final Command rc_retractIntake = new InstantCommand(rc_intakesub::retractIntake, rc_intakesub);
  //limeLight
    private final LimelightSub rc_limelightsub = new LimelightSub();
      //LimelightShot
      private final Command rc_limelightShotDrive = new RunCommand(rc_drivetrainsub::aim, rc_drivetrainsub);
      private final Command rc_limelightShotSpinShooter = new RunCommand(rc_shootersub::limelightShot, rc_shootersub);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //set default commands
    rc_shootersub.setDefaultCommand(rc_idleshooter);
    rc_drivetrainsub.setDefaultCommand(rc_drive);
  
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
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  */
}
