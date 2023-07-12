package frc.robot.Subsystems;

//imports resources
import frc.robot.Robot;

import java.util.ArrayList;
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
public class ExampleSimpleArm extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //  defines and instantiates motors, pid controllers, and encoders

  private CANSparkMax leftArm = Robot.hardware.leftArm;
  private CANSparkMax rightArm = Robot.hardware.rightArm;
  private CANSparkMax translateArm = Robot.hardware.transArm;

  private SparkMaxPIDController leftArmPID = Robot.hardware.leftArmPID;
  private SparkMaxPIDController rightArmPID = Robot.hardware.rightArmPID;
  private SparkMaxPIDController transArmPID = Robot.hardware.transArmPID;

  private RelativeEncoder leftArmEnc = Robot.hardware.leftArmEnc;
  private RelativeEncoder rightArmEnc = Robot.hardware.leftArmEnc;

  public class ArmState {
    //defines variables for positions
    private double radialPos;
    private double transPos;

    public ArmState(double radialPos, double transPos){
      //sets the arm's radial and transversal posititons
      this.radialPos = radialPos;
      this.transPos = transPos;
    };
    
  }

    //defines variable for the class above
  public ArmState currentArmState; 


  Map<String, ArmState> armStates = Map.ofEntries(
    // sets the 2 states for the arm
                    Map.entry("DEFAULT", new ArmState(0, 0)),
                    Map.entry("EXTENDED", new ArmState(5, 10))
                );
        
  public ExampleSimpleArm() {
    //makes the left arm inverted but following the right arm
    leftArm.setInverted(true);
    leftArm.follow(rightArm);
  }

  public void setPivotSpeed(double power) {
    //sets the power to the arm motors
    rightArm.set(power);
    //outputs the motors power to the smart dashboard
    SmartDashboard.putNumber("Left Arm Output ", power);
    SmartDashboard.putNumber( "Right Arm Output ", power);    
}

  public void setTranslateSpeed(double power) {
    //sets the power to the translate motor
    translateArm.set(power);
    //outputs the motor power to the smart dashboard
    SmartDashboard.putNumber( "Translate Arm Output ", power);    
  }

  public void setPositions(ArmState armState){
    //instansiates current arm state
    currentArmState = armState;
    //defines and instantiates setpoins
    double rotSetpoint = armState.radialPos;
    double transSetpoint = armState.transPos;

    //sets motor pid references
    rightArmPID.setReference(rotSetpoint, ControlType.kSmartMotion);
    leftArmPID.setReference(rotSetpoint, ControlType.kSmartMotion);
    transArmPID.setReference(transSetpoint, ControlType.kSmartMotion);

    //outputs the pid references to the smart dashboard
    SmartDashboard.putNumber("Right Arm Setpoint", rotSetpoint);
    SmartDashboard.putNumber("Left Arm Setpoint", rotSetpoint);
    SmartDashboard.putNumber("Trans Arm Setpoint", transSetpoint);
  }


  //method to get the current arm state
  public ArmState getArmState(){
    return currentArmState;
  }
  //makes a list of the arm positions
  public List<Double> getPositions(){
    List<Double> positions = new ArrayList<Double>(3);
    positions.add(leftArmEnc.getPosition());
    positions.add(rightArmEnc.getPosition());

    //Average velocity of the two shooter motors for indication of shared axle problems
    positions.add((rightArmEnc.getPosition() + leftArmEnc.getPosition()) / 2); 
     return positions;
  }


}