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
public class ExampleSimpleElevator extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax leftElevatorMotor = Robot.hardware.leftElevatorMotor;
  private CANSparkMax rightElevatorMotor = Robot.hardware.rightElevatorMotor;

  private SparkMaxPIDController leftElevatorPID = Robot.hardware.leftElevatorPID;
  private SparkMaxPIDController rightElevatorPID = Robot.hardware.rightElevatorPID;

  private RelativeEncoder leftElevatorEnc = Robot.hardware.leftElevatorEnc;
  private RelativeEncoder rightElevatorEnc = Robot.hardware.rightElevatorEnc;


  public class ElevatorState {
    private double leftPos;
    private double rightPos;

    public ElevatorState(double leftPos, double rightPos){
      this.leftPos = leftPos;
      this.rightPos = rightPos;
    };
    
  }

  public ElevatorState currentElevatorState; 


  Map<String, ElevatorState> elevatorStates = Map.ofEntries(
                    Map.entry("DEFAULT", new ElevatorState(0, 0)),
                    Map.entry("LOW_HEIGHT", new ElevatorState(5, 5)), 
                    Map.entry("LOW_HEIGHT", new ElevatorState(10, 10))

                );
        
  public ExampleSimpleElevator() {
    
    leftElevatorMotor.setInverted(true);
    leftElevatorMotor.follow(rightElevatorMotor);
  }

  public void setPivotSpeed(double power) {
    rightElevatorMotor.set(power);
    SmartDashboard.putNumber("Left Arm Output ", power);
    SmartDashboard.putNumber( "Right Arm Output ", power);    
}

  public void setPositions(ElevatorState elevatorState){
    currentElevatorState = elevatorState;
    double leftSetpoint = elevatorState.leftPos;
    double rightSetpoint = elevatorState.rightPos;

    leftElevatorPID.setReference(leftSetpoint, ControlType.kSmartMotion);
    rightElevatorPID.setReference(rightSetpoint, ControlType.kSmartMotion);

    SmartDashboard.putNumber("Right Setpoint", rightSetpoint);
    SmartDashboard.putNumber("Left Setpoint", leftSetpoint);
  }


  public ElevatorState getElevatorState(){
    return currentElevatorState;
  }
  public List<Double> getPositions(){
    List<Double> positions = new ArrayList<Double>(3);
    positions.add(leftElevatorEnc.getPosition());
    positions.add(rightElevatorEnc.getPosition());

    //Average velocity of the two shooter motors for indication of shared axle problems
    positions.add((rightElevatorEnc.getPosition() + leftElevatorEnc.getPosition()) / 2); 
     return positions;
  }


}