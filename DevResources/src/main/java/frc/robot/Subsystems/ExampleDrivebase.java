package frc.robot.Subsystems;

import frc.robot.Robot;
import frc.robot.Utilities.SparkMAXMotorGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class ExampleDrivebase extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private SparkMAXMotorGroup leftDrives = Robot.hardware.leftDriveMotors;
  private SparkMAXMotorGroup rightDrives = Robot.hardware.rightDriveMotors;
  
  

  public ExampleDrivebase() {
    
    leftDrives.setInverted(true);
    rightDrives.setInverted(false);

  }

  public void setPercentOutput(double leftPower, double rightPower) {
    leftDrives.set(leftPower);
    rightDrives.set(rightPower);
    SmartDashboard.putNumber("Left Drive Output ", leftPower);
    SmartDashboard.putNumber("Right Drive Output ", rightPower);
}




}