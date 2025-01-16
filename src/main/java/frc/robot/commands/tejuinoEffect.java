// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.TejuinoSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class tejuinoEffect extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TejuinoSubsystem tejuinoSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public tejuinoEffect(TejuinoSubsystem tejuinoSubsystem) {
    this.tejuinoSubsystem = tejuinoSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tejuinoSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
System.out.println("The command is starting");
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    tejuinoSubsystem.tRams_effect(1);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
