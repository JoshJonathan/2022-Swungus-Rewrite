// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.PS4ControllerSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSub rc_IntakeSub = new IntakeSub();

  private final Command deployIntake = new InstantCommand(rc_IntakeSub::deployIntake, rc_IntakeSub);
  private final Command retractIntake = new InstantCommand(rc_IntakeSub::retractIntake, rc_IntakeSub);

  private final ShooterSub rc_ShooterSub = new ShooterSub();

  private final Command idleShooter = new InstantCommand(rc_ShooterSub::idle, rc_ShooterSub);
 
  PS4Controller rc_operatorController = new PS4Controller(Constants.OPERATOR_CONTROLLER_PORT);
  PS4ControllerSim rc_operatorControllerSim = new PS4ControllerSim(rc_operatorController);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //rc_ShooterSub.setDefaultCommand(idleShooter);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(rc_operatorController, PS4Controller.Button.kR1.value)
        .whenPressed(deployIntake);
    new JoystickButton(rc_operatorController, PS4Controller.Button.kL1.value)
        .whenPressed(retractIntake);
    new JoystickButton(rc_operatorController, PS4Controller.Button.kCross.value)
        .whenPressed(idleShooter);
  }
}
