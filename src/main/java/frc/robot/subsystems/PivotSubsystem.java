// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

    // Motor and encoder
    private final SparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;

    // PID controller for position control
    private final PIDController pivotPID;

    // Target position for the pivot (in encoder units)
    private double targetPosition = 0.0;

    public PivotSubsystem() {
        // Initialize motor and encoder
        pivotMotor = new SparkMax(3,com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

        // Reset encoder position
        pivotEncoder.setPosition(0.0);

        // Initialize the PID controller with P, I, D values (these need to be tuned for your setup)
        pivotPID = new PIDController(0.015, 0.0007, 0.001);
        
        
    }

    // Method to set a target position
    public void setTargetPosition(double position) {
        targetPosition = position;
    }

    // Preset positions
    public void moveToPositionCero() {
        setTargetPosition(0.0);  // Replace with actual position
    }

    public void moveToPositionOne() {
        setTargetPosition(10.0);  // Replace with actual position
    }

    @Override
    public void periodic() {
        // Calculate the output power based on the current encoder position and target position
        double pidOutput = pivotPID.calculate(pivotEncoder.getPosition(), targetPosition);

        // Set the motor power to the PID output
        pivotMotor.set(pidOutput);

       
    }

    // Method to check if the motor has reached the target position
    public boolean atTargetPosition() {
        return pivotPID.atSetpoint();
    }
}