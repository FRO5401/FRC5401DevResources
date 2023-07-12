package frc.robot.Subsystems;

// imports resources
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

  //defines and instantiates motors, pid controllers, and encoders
  private CANSparkMax leftShooterMotor = Robot.hardware.leftShooterMotor;
  private CANSparkMax rightShooterMotor = Robot.hardware.rightShooterMotor;


  private SparkMaxPIDController leftShooterPID = Robot.hardware.leftShooterPID;
  private SparkMaxPIDController rightShooterPID = Robot.hardware.rightShooterPID;

  private RelativeEncoder leftShooterEnc = Robot.hardware.leftShooterEnc;
  private RelativeEncoder rightShooterEnc = Robot.hardware.rightShooterEnc;

  public class ShooterState {
    // defines variables for speed in RPM and perfent
    private double shooterSpeedRPM;
    private double shooterSpeedPercent;

    public ShooterState(double shooterSpeedRPM, double shooterSpeedPercent){
      //sets the shooter's rpm and percent speed
      // What is the point of this method
      this.shooterSpeedRPM = shooterSpeedRPM;
      this.shooterSpeedPercent = shooterSpeedPercent;

    };
    
  }

  //defines variable for the method above
  public ShooterState currentShooterState; 


  Map<String, ShooterState> armStates = Map.ofEntries(
    // sets the 3 modes of the shooter
                    Map.entry("LOW", new ShooterState(6500, 0.35)),
                    Map.entry("MEDIUM", new ShooterState(9500, 0.60)),
                    Map.entry("HIGH", new ShooterState(11500, 0.8))
                );
        
  public ExampleShooter() {
    //sets the left shooter to follow but do the opposite of the right shooter
    leftShooterMotor.setInverted(true);
    leftShooterMotor.follow(rightShooterMotor);
  }

  public void setShooterSpeed(double power) {
    //a method to set the shooter power
    rightShooterMotor.set(power);
    SmartDashboard.putNumber("Left Shooter Output ", power);
    SmartDashboard.putNumber( "Right Shooter Output ", power);  
      
}

  public void setSpeeds(ShooterState shooterState){
    //a method to set the speeds of the motor
    currentShooterState = getShooterState();
    //why is unused variable here, also why define and instantiate in one line
    double setpointRPM = currentShooterState.shooterSpeedRPM;
    double setpointPercent = currentShooterState.shooterSpeedPercent;

    //what does this line mean
    rightShooterPID.setReference(setpointPercent, ControlType.kSmartMotion);

    SmartDashboard.putNumber("Left Shooter Setpoint", setpointPercent);
    SmartDashboard.putNumber("Right Shooter Setpoint", setpointPercent);
  }


  public ShooterState getShooterState(){
    //gets current shooter state
    return currentShooterState;
  }
  public List<Double> getSpeeds(){
    //makes a list of speeds for the shooter
    List<Double> speeds = new ArrayList<Double>(3);
    speeds.add(leftShooterEnc.getVelocity());
    speeds.add(rightShooterEnc.getVelocity());

    //Average velocity of the two shooter motors for indication of shared axle problems
    speeds.add((rightShooterEnc.getVelocity() + leftShooterEnc.getVelocity()) / 2); 
     return speeds;
  }

}