// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.RobotContainer;


public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

   VictorSP leftSide = new VictorSP(Constants.DrivetrainConstants.leftSideConstant);
   VictorSP rightSide = new VictorSP(Constants.DrivetrainConstants.rightSideConstant);

  private final Encoder l_encoder = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);
  private final Encoder r_encoder = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
 
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI);//No sé si esté bien esto ojo//



  
  private final DifferentialDriveOdometry m_odometry;

  DifferentialDrive differentialDrive = new DifferentialDrive(leftSide, rightSide);

  public DrivetrainSubsystem() {
l_encoder.reset();
r_encoder.reset();

rightSide.setInverted(true);
leftSide.setInverted(false);

m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), l_encoder.getDistance(), r_encoder.getDistance());

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
public void arcadeDrive(double fwd, double rot){
differentialDrive.arcadeDrive(fwd, rot);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(navX.getRotation2d(), 0, 0);
    // Log encoder values
    
    

    // Update odometry
    m_odometry.update(navX.getRotation2d(), l_encoder.getDistance(), r_encoder.getDistance());
  }


}
