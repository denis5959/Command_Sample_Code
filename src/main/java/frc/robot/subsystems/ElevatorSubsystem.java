// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase {

    // Motor and encoder
    private final SparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;

    // PID controller for position control
    private final PIDController pivotPID;

    // Target position for the pivot (in encoder units)
    private double targetPosition = 0.0;

    private double customOutput = 0.0;
    private boolean isManualControl = false;
    //

    public ElevatorSubsystem() {
        // Initialize motor and encoder
        pivotMotor = new SparkMax(3,com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

        // Reset encoder position
        pivotEncoder.setPosition(0.0);

        // Initialize the PID controller with P, I, D values (these need to be tuned for your setup)
        pivotPID = new PIDController(0.037, 0.0008, 0.008);
        
        
    }
    public void holdCurrentPosition() {
        isManualControl = false;
        targetPosition = pivotEncoder.getPosition(); // Set target to current position
    }

    // Method to increment the motor manually
    public void increment(double customOutput) {
        isManualControl = true;
        pivotMotor.set(customOutput); // Run motor with custom output
    }

    // Preset positions
    public void moveToPositionCero() {
        isManualControl = false; // Disable manual control for preset position
        setTargetPosition(0.0);  // Move to preset position 0
    }

    public void moveToPositionOne() {
        isManualControl = false; // Disable manual control for preset position
        setTargetPosition(20.0);  // Move to preset position 1
    }

    public void moveToPositionTwo() {
        isManualControl = false; // Disable manual control for preset position
        setTargetPosition(40.0);  // Move to preset position 2
    }

    // Method to set a target position
    public void setTargetPosition(double position) {
        targetPosition = position;
    }

    @Override
    public void periodic() {
        if (!isManualControl) {
            // PID control mode
            double pidOutput = pivotPID.calculate(pivotEncoder.getPosition(), targetPosition);
            pivotMotor.set(pidOutput); // Set the motor to the calculated PID output
        }
       
    }

    // Method to check if the motor has reached the target position
    public boolean atTargetPosition() {
        return pivotPID.atSetpoint();
    }
}