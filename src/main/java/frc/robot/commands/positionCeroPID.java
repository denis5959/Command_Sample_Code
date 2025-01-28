// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class positionCeroPID extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem pivotSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public positionCeroPID(ElevatorSubsystem pivotSubsystem) {
    this.pivotSubsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
System.out.println("The command is starting");
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   
/*if(RobotContainer.controlcito.getR2Button()){
  pivotSubsystem.increment(((RobotContainer.controlcito.getR2Axis())+1)/2);
} else { */
  if (RobotContainer.controlcito.getCrossButtonPressed()) {
    pivotSubsystem.moveToPositionCero();
}
if (RobotContainer.controlcito.getCircleButtonPressed()) {
    pivotSubsystem.moveToPositionOne();
}
if (RobotContainer.controlcito.getTriangleButton()) {
  pivotSubsystem.moveToPositionTwo();
}
  
  
/* else{

  pivotSubsystem.holdCurrentPosition();
}}*/
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