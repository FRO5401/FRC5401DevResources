package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Utilities.SparkMAXMotorGroup;
import frc.robot.Utilities.SparkMaxPIDConfig;

public class Hardware {
    //Example drivebase hardware
    public CANSparkMax leftDrive1; //Methods for the 4 motors on the robot
	public CANSparkMax leftDrive2;
	public CANSparkMax rightDrive1;
	public CANSparkMax rightDrive2;

    public RelativeEncoder leftDrive1Enc; //Methods for the claw encoders
    public RelativeEncoder leftDrive2Enc;
    public RelativeEncoder rightDrive1Enc;
    public RelativeEncoder rightDrive2Enc;

    public SparkMAXMotorGroup leftDriveMotors; //Method for the 2 motors on the left side of the robot
    public SparkMAXMotorGroup rightDriveMotors; //Method for the 2 motors on the right side of the robot

    //Example claw hardware
    public Solenoid firstClawStage; //Methods for the 2 claw stages
    public Solenoid secondClawStage; 

    //Example arm hardware
    public CANSparkMax leftArm; //Methods for the different parts of the arm
    public CANSparkMax rightArm;
    public CANSparkMax transArm;

    public SparkMaxPIDConfig leftArmPIDConfig; //Methods for the arm motors during autonomous
    public SparkMaxPIDConfig rightArmPIDConfig;
    public SparkMaxPIDConfig transArmPIDConfig;

    public SparkMaxPIDController leftArmPID; //Methods for the arm motors during autonomous
    public SparkMaxPIDController rightArmPID;
    public SparkMaxPIDController transArmPID;

    public RelativeEncoder leftArmEnc; //Methods for the arm encoders
    public RelativeEncoder rightArmEnc;
    public RelativeEncoder transArmEnc;

    //Example shooter hardware
    public CANSparkMax leftShooterMotor; //Methods for the shooter motors
    public CANSparkMax rightShooterMotor;

    public SparkMaxPIDController leftShooterPID; //Methods for the shooter motors during autonomous
    public SparkMaxPIDController rightShooterPID;

    public RelativeEncoder leftShooterEnc; //Methods for the shooter encoders
    public RelativeEncoder rightShooterEnc;

    //Example elevator hardware
    public CANSparkMax rightElevatorMotor; //Methods for the elevator motors
    public CANSparkMax leftElevatorMotor;

    public SparkMaxPIDController leftElevatorPID; //Methods for the elevator motors during autonomous
    public SparkMaxPIDController rightElevatorPID;

    public RelativeEncoder leftElevatorEnc; //Methods for the elevator encoders
    public RelativeEncoder rightElevatorEnc;
  

   


    public Hardware() {
        //Drivebase hardware
        leftDrive1 = new CANSparkMax(0, MotorType.kBrushless); //Initiate the 4 motors on the drivebase
        leftDrive2 = new CANSparkMax(1, MotorType.kBrushless);
        rightDrive1 = new CANSparkMax(2, MotorType.kBrushless);
        rightDrive2 = new CANSparkMax(3, MotorType.kBrushless);

        leftDrive1Enc = leftDrive1.getEncoder(); //Initiate the 4 encoders
        leftDrive2Enc = leftDrive2.getEncoder();
        rightDrive1Enc = rightDrive1.getEncoder();
        rightDrive2Enc = rightDrive2.getEncoder();

        leftDriveMotors = new SparkMAXMotorGroup("Left Drive Motor Group", leftDrive1, leftDrive2); //Initiate the 2 motors on the left
        rightDriveMotors = new SparkMAXMotorGroup("Right Drive Motor Group", rightDrive1, rightDrive2); //Initiate the 2 motors on the right

        leftDriveMotors.setIdleMode(IdleMode.kBrake);
        rightDriveMotors.setIdleMode(IdleMode.kBrake);

        //Claw hardware
        firstClawStage = new Solenoid(PneumaticsModuleType.CTREPCM, 1); //Initiate when the claw changes actions
        secondClawStage = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

        //Simple arm hardware
        leftArm = new CANSparkMax(4, MotorType.kBrushless); //Initiate the motors on the arm
        rightArm = new CANSparkMax(5, MotorType.kBrushless);
        transArm = new CANSparkMax(6, MotorType.kBrushless);

        leftArmPIDConfig = new SparkMaxPIDConfig("Left Arm PID Controller", leftArm.getPIDController(), 0, 0, 0, 0); //Initiate the arm PID controllers
        rightArmPIDConfig = new SparkMaxPIDConfig("Right Arm PID Controller", rightArm.getPIDController(), 0, 0, 0, 0);
        transArmPIDConfig = new SparkMaxPIDConfig("Trans Arm PID Controller", transArm.getPIDController(), 0, 0, 0, 0);

        leftArmPID = leftArmPIDConfig.getConfPIDController(); //Controls the motors during autonomous
        rightArmPID = rightArmPIDConfig.getConfPIDController();
        transArmPID = transArmPIDConfig.getConfPIDController();
        
        leftArmEnc = leftArm.getEncoder(); 
        rightArmEnc = rightArm.getEncoder();
        transArmEnc = transArm.getEncoder();

        //Simple elevator hardware
        leftElevatorMotor = new CANSparkMax(6, MotorType.kBrushless); //Initiates the elevator motors
        rightElevatorMotor = new CANSparkMax(7, MotorType.kBrushless);

        leftArmPIDConfig = new SparkMaxPIDConfig("Left Elevator Motor PID Controller", leftArm.getPIDController(), 0, 0, 0, 0); //Initiate the arm PID controllers
        rightArmPIDConfig = new SparkMaxPIDConfig("Right Elevator Motor PID Controller", rightArm.getPIDController(), 0, 0, 0, 0);

        leftArmPID = leftArmPIDConfig.getConfPIDController(); //Controls the motors during autonomous
        rightArmPID = rightArmPIDConfig.getConfPIDController();
        transArmPID = transArmPIDConfig.getConfPIDController();
        
        leftArmEnc = leftArm.getEncoder();
        rightArmEnc = rightArm.getEncoder();
        transArmEnc = rightArm.getEncoder();


    }


    



}
