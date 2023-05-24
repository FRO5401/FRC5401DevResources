package frc.robot.Subsystems;

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

  private SparkMAXMotorGroup leftDrives = Robot.hardware.leftDriveMotors;
  private SparkMAXMotorGroup rightDrives = Robot.hardware.rightDriveMotors;
  
  private RelativeEncoder leftDrive1Enc = Robot.hardware.leftDrive1Enc;
  private RelativeEncoder leftDrive2Enc = Robot.hardware.leftDrive2Enc;
  private RelativeEncoder rightDrive1Enc = Robot.hardware.rightDrive1Enc;
  private RelativeEncoder rightDrive2Enc = Robot.hardware.rightDrive2Enc;


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

    navX = new AHRS();
    navX.reset();

    turn_kP = 1f/30f; 
    turn_kI = 0; 
    turn_kD = 0;
    turn_tolerance = 1;
    turn_derivativeTolerance = .01;

    turn_error = -navX.getRate();
    turn_PID = new PIDController(turn_kP, turn_kI, turn_kD);
    turn_PID.setTolerance(turn_tolerance, turn_derivativeTolerance);
    
    leftDrives.setInverted(true);
    rightDrives.setInverted(false);

  }

  public void setPercentOutput(double leftPower, double rightPower) {
    leftDrives.set(leftPower);
    rightDrives.set(rightPower);
    SmartDashboard.putNumber("Left Drive Output ", leftPower);
    SmartDashboard.putNumber("Right Drive Output ", rightPower);
  }

  public void driveToHeading(double targetAngle)
  {
    double calculatedPID = turn_PID.calculate(getAngleDouble(), targetAngle);
    leftDrives.set(calculatedPID);
    rightDrives.set(-calculatedPID);
  }

  // Tested: Negative, negative
  public void driveByVolts(double leftVolts, double rightVolts) {
    leftDrives.setVoltages(-leftVolts);
    rightDrives.setVoltages(-rightVolts);
  }


  public void resetGyro(){
    navX.calibrate();
  }

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(navX.getAngle());
    
  }
  public double getAngleDouble(){
    return navX.getAngle();
    
  }

  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    navX.reset();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetEncoders()
  {
    leftDrive1Enc.setPosition(0);
    leftDrive2Enc.setPosition(0);
    rightDrive1Enc.setPosition(0);
    rightDrive2Enc.setPosition(0);

  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(getAngle(), leftDrive1Enc.getPosition(), rightDrive1Enc.getPosition(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftDrive1Enc.getVelocity(), rightDrive1Enc.getVelocity());
  }



}