// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
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

    //Elevator
    private final ElevatorSub rc_elevatorsub = new ElevatorSub();
    private final Command rc_elevatorIdle = new InstantCommand(rc_elevatorsub::periodic);
    private final Command rc_elevatorUp = new InstantCommand(rc_elevatorsub::upButton);
    private final Command rc_elevatorDown = new InstantCommand(rc_elevatorsub::downButton);
    private final Command rc_elevatorBrake = new InstantCommand(rc_elevatorsub::brakeElevator);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


//rc_elevatorsub.setDefaultCommand(rc_elevatorsub,rc_elevatorIdle);
   // rc_elevatorsub.setDefaultCommand(rc_elevatorDefault);
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
      new JoystickButton(rc_driverController, XboxController.Button.kA.value).whenPressed(rc_elevatorDown);
      new JoystickButton(rc_driverController, XboxController.Button.kB.value).whenPressed(rc_elevatorUp);
      new JoystickButton(rc_driverController, XboxController.Button.kY.value).whenPressed(rc_elevatorBrake);
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
