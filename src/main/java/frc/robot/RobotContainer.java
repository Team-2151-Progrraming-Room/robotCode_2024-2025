// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.CoralSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer{ 
  // Subsystems

  private final CoralSubsystem Coral;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick buttonBoard = new Joystick(1);

  public final JoystickButton button1;
  public final JoystickButton button2;
  public final JoystickButton button3;
  public final JoystickButton button4;
  public final JoystickButton button5;
  public final JoystickButton button6;
  public final JoystickButton button7;
  public final JoystickButton button8;
  public final JoystickButton button9;
  public final JoystickButton button10;
  public final JoystickButton button11;
  public final JoystickButton button12;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // subsystems
    Coral = new CoralSubsystem();
    //buttons that will allow us to link different commands to each button
    button1 = new JoystickButton(buttonBoard, 1);
    button2 = new JoystickButton(buttonBoard, 2);
    button3 = new JoystickButton(buttonBoard, 3);
    button4 = new JoystickButton(buttonBoard, 4);
    button5 = new JoystickButton(buttonBoard, 5);
    button6 = new JoystickButton(buttonBoard, 6);
    button7 = new JoystickButton(buttonBoard, 7);
    button8 = new JoystickButton(buttonBoard, 8);
    button9 = new JoystickButton(buttonBoard, 9);
    button10 = new JoystickButton(buttonBoard, 10);
    button11 = new JoystickButton(buttonBoard, 11);
    button12 = new JoystickButton(buttonBoard, 12);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines


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
    // Default command, normal field-relative drive
    button3.whileTrue(Coral.coralMotorOnCommand());
   // button4.onTrue();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
