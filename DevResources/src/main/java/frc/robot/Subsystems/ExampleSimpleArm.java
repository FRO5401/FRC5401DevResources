package frc.robot.Subsystems;

import frc.robot.Robot;
import frc.robot.Utilities.SparkMAXMotorGroup;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class ExampleSimpleArm extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax leftArm = Robot.hardware.leftArm;
  private CANSparkMax rightArm = Robot.hardware.rightArm;
  private CANSparkMax translateArm = Robot.hardware.translateArm;

  

  public ExampleSimpleArm() {
    
    leftArm.setInverted(true);
    rightArm.setInverted(false);

  }

  public void setPivotSpeed(double power) {
    leftArm.set(power);
    rightArm.set(power);
    SmartDashboard.putNumber("Left Drive Output ", power);
    SmartDashboard.putNumber( "Right Drive Output ", power);    
}

  public void setTranslateSpeed(double power) {
    translateArm.set(power);
    SmartDashboard.putNumber( "Translate Output ", power);    
  }






}