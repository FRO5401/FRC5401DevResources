package frc.robot.Subsystems;

import frc.robot.Robot;
import frc.robot.Utilities.SparkMAXMotorGroup;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class ExampleShooter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax leftShooterMotor = Robot.hardware.leftShooterMotor;
  private CANSparkMax rightShooterMotor = Robot.hardware.rightShooterMotor;

  private SparkMaxPIDController leftShooterPID = Robot.hardware.leftShooterPID;
  private SparkMaxPIDController rightShooterPID = Robot.hardware.rightShooterPID;

  private RelativeEncoder leftShooterEnc = Robot.hardware.leftShooterEnc;
  private RelativeEncoder rightShooterEnc = Robot.hardware.rightShooterEnc;

  public class ShooterState {
    private double shooterSpeedRPM;
    private double shooterSpeedPercent;

    public ShooterState(double shooterSpeedRPM, double shooterSpeedPercent){
      this.shooterSpeedRPM = shooterSpeedRPM;
      this.shooterSpeedPercent = shooterSpeedPercent;

    };
    
  }

  public ShooterState currentShooterState; 


  Map<String, ShooterState> armStates = Map.ofEntries(
                    Map.entry("LOW", new ShooterState(6500, 0.35)),
                    Map.entry("MEDIUM", new ShooterState(9500, 0.60)),
                    Map.entry("HIGH", new ShooterState(11500, 0.8))
                );
        
  public ExampleShooter() {
    leftShooterMotor.setInverted(true);
    leftShooterMotor.follow(rightShooterMotor);
  }

  public void setShooterSpeed(double power) {
    rightShooterMotor.set(power);
    SmartDashboard.putNumber("Left Shooter Output ", power);
    SmartDashboard.putNumber( "Right Shooter Output ", power);  
      
}

  public void setPosition(ShooterState shooterState){
    currentShooterState = shooterState;
    double rotSetpoint = currentShooterState.shooterSpeedPercent;
    double transSetpoint = currentShooterState.shooterSpeedRPM;

    rightShooterPID.setReference(rotSetpoint, ControlType.kSmartMotion);
    leftShooterPID.setReference(rotSetpoint, ControlType.kSmartMotion);

    SmartDashboard.putNumber("Right Arm Setpoint", rotSetpoint);
    SmartDashboard.putNumber("Left Arm Setpoint", rotSetpoint);
    SmartDashboard.putNumber("Trans Arm Setpoint", transSetpoint);
  }


  public ShooterState getArmState(){
    return currentShooterState;
  }
  public List<Double> getSpeeds(){
    List<Double> speeds = new ArrayList<Double>(3);
    speeds.add(leftShooterEnc.getVelocity());
    speeds.add(rightShooterEnc.getVelocity());

    //Average velocity of the two shooter motors for indication of shared axle problems
    speeds.add((rightShooterEnc.getVelocity() + leftShooterEnc.getVelocity()) / 2); 
     return speeds;
  }

}