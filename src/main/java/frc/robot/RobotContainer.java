// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.PivotConstants;
import frc.robot.commands.driveWithJoystick;
import frc.robot.commands.positionCeroPID;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TejuinoSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final driveWithJoystick m_exampleCommand = new driveWithJoystick(drivetrainSubsystem);
  private final TejuinoSubsystem tejuino = new TejuinoSubsystem();
  public final static PS4Controller controlcito = new PS4Controller(0);
  

private final ElevatorSubsystem pivotSubsystem = new ElevatorSubsystem();
  private final positionCeroPID PIDstate = new positionCeroPID(pivotSubsystem);

  /** The container for the robot. Contalllins subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    drivetrainSubsystem.setDefaultCommand(m_exampleCommand);
    pivotSubsystem.setDefaultCommand(PIDstate);
    
  }

 private void configureButtonBindings(){


 }
  public Command getAutonomousCommand() {
    return m_exampleCommand;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 
}
