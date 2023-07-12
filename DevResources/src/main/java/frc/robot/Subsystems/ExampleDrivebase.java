package frc.robot.Subsystems;

//imports resources
import frc.robot.Robot;
import frc.robot.Utilities.SparkMAXMotorGroup;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class ExampleDrivebase extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // defines and instantiates variables for motor groups and encoders
  ///??? QUESTION SHOULD THESE BE SPLIT INTO DEFINITIONS THEN INSTANTIAIONS???
  private SparkMAXMotorGroup leftDrives = Robot.hardware.leftDriveMotors;
  private SparkMAXMotorGroup rightDrives = Robot.hardware.rightDriveMotors;
  
  private RelativeEncoder leftDrive1Enc = Robot.hardware.leftDrive1Enc;
  private RelativeEncoder leftDrive2Enc = Robot.hardware.leftDrive2Enc;
  private RelativeEncoder rightDrive1Enc = Robot.hardware.rightDrive1Enc;
  private RelativeEncoder rightDrive2Enc = Robot.hardware.rightDrive2Enc;

  //defines variabes 
  private final AHRS navX;
  private final PIDController turn_PID;
  private DifferentialDriveOdometry odometry;

  public double turn_kP;
  public double turn_kI;
  public double turn_kD;
  public double turn_tolerance;
  public double turn_derivativeTolerance;
  public double turn_error;
  

  public ExampleDrivebase() {

    // instantiates gyro
    navX = new AHRS();
    //resets gyro
    navX.reset();

    //instantiates variables ????what do the variables do??
    turn_kP = 1f/30f; 
    turn_kI = 0; 
    turn_kD = 0;
    turn_tolerance = 1;
    turn_derivativeTolerance = .01;

    turn_error = -navX.getRate();
    turn_PID = new PIDController(turn_kP, turn_kI, turn_kD);
    turn_PID.setTolerance(turn_tolerance, turn_derivativeTolerance);
    
    //inverts the controls so it is equal both sides
    leftDrives.setInverted(true);
    rightDrives.setInverted(false);

  }

  public void setPercentOutput(double leftPower, double rightPower) {
    //method to control the power output of the motors
    leftDrives.set(leftPower);
    rightDrives.set(rightPower);
    SmartDashboard.putNumber("Left Drive Output ", leftPower);
    SmartDashboard.putNumber("Right Drive Output ", rightPower);
  }

  public void driveToHeading(double targetAngle) {
    //method to drive at a specific angle
    double calculatedPID = turn_PID.calculate(getAngleDouble(), targetAngle);
    leftDrives.set(calculatedPID);
    rightDrives.set(-calculatedPID);
  }

  // Tested: Negative, negative
  public void driveByVolts(double leftVolts, double rightVolts) {
    //method to drive by voltages????
    leftDrives.setVoltages(-leftVolts);
    rightDrives.setVoltages(-rightVolts);
  }


  public void resetGyro(){
    //method to recalibrate the gyro
    navX.calibrate();
  }

  public Rotation2d getAngle(){
    //method to get the current angle in degrees from the gyro
    return Rotation2d.fromDegrees(navX.getAngle());
    
  }
  public double getAngleDouble(){
    //method to get the current angle as a double from the gyro
    return navX.getAngle();
    
  }

  public double getHeading() {
    //method to get the degrees?????
    return navX.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    //method to reset the gyro???
    navX.reset();
  }

  public double getTurnRate() {
    //idk gets the rate of how often it turns?
    return -navX.getRate();
  }

  public Pose2d getPose(){
    // idk gets position?
    return odometry.getPoseMeters();
  }

  public void resetEncoders()
  {
    //sets all encoders to 0
    leftDrive1Enc.setPosition(0);
    leftDrive2Enc.setPosition(0);
    rightDrive1Enc.setPosition(0);
    rightDrive2Enc.setPosition(0);

  }

  public void resetOdometry(Pose2d pose){
    //idk resets position?
    resetEncoders();
    odometry.resetPosition(getAngle(), leftDrive1Enc.getPosition(), rightDrive1Enc.getPosition(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //gets the speeds of the wheels
    return new DifferentialDriveWheelSpeeds(leftDrive1Enc.getVelocity(), rightDrive1Enc.getVelocity());
  }



}