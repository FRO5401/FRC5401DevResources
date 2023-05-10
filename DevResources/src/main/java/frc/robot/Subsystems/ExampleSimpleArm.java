package frc.robot.Subsystems;

import frc.robot.Robot;
import frc.robot.Utilities.SparkMAXMotorGroup;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

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
  private CANSparkMax translateArm = Robot.hardware.transArm;

  private SparkMaxPIDController leftArmPID = Robot.hardware.leftArmPID;
  private SparkMaxPIDController rightArmPID = Robot.hardware.rightArmPID;
  private SparkMaxPIDController transArmPID = Robot.hardware.transArmPID;

  public class ArmState {
    private double radialPos;
    private double transPos;

    public ArmState(double radialPos, double transPos){
      this.radialPos = radialPos;
      this.transPos = transPos;
    };
    
  }

  public ArmState currentArmState; 


  Map<String, ArmState> armStates = Map.ofEntries(
                    Map.entry("DEFAULT", new ArmState(0, 0)),
                    Map.entry("EXTENDED", new ArmState(5, 10))
                );
        
  public ExampleSimpleArm() {
    
    leftArm.setInverted(true);
    leftArm.follow(rightArm);
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

  public void setPosition(ArmState armState){
    currentArmState = armState;
    double rotSetpoint = armState.radialPos;
    double transSetpoint = armState.transPos;

    rightArmPID.setReference(rotSetpoint, ControlType.kSmartMotion);
    leftArmPID.setReference(rotSetpoint, ControlType.kSmartMotion);
    transArmPID.setReference(transSetpoint, ControlType.kSmartMotion);

    SmartDashboard.putNumber("Right Arm Setpoint", rotSetpoint);
    SmartDashboard.putNumber("Left Arm Setpoint", rotSetpoint);
    SmartDashboard.putNumber("Trans Arm Setpoint", transSetpoint);
  }


  public ArmState getArmState(){
    return currentArmState;
  }
  public double[] getPosition(){

    double[] position = {};
    //position[0] = rightArmPID.getPosition();
    //position[1] = rightArmPID.getPosition();
    //position[2] = rightArmPID.getPosition();
  
     return (double[]) position;
  }

}