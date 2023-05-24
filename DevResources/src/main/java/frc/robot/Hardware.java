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
    public CANSparkMax leftDrive1;
	public CANSparkMax leftDrive2;
	public CANSparkMax rightDrive1;
	public CANSparkMax rightDrive2;

    public RelativeEncoder leftDrive1Enc;
    public RelativeEncoder leftDrive2Enc;
    public RelativeEncoder rightDrive1Enc;
    public RelativeEncoder rightDrive2Enc;

    public SparkMAXMotorGroup leftDriveMotors;
    public SparkMAXMotorGroup rightDriveMotors;

    //Example claw hardware
    public Solenoid firstClawStage; 
    public Solenoid secondClawStage; 

    //Example arm hardware
    public CANSparkMax leftArm;
    public CANSparkMax rightArm;
    public CANSparkMax transArm;

    public SparkMaxPIDConfig leftArmPIDConfig;
    public SparkMaxPIDConfig rightArmPIDConfig;
    public SparkMaxPIDConfig transArmPIDConfig;

    public SparkMaxPIDController leftArmPID;
    public SparkMaxPIDController rightArmPID;
    public SparkMaxPIDController transArmPID;

    public RelativeEncoder leftArmEnc;
    public RelativeEncoder rightArmEnc;
    public RelativeEncoder transArmEnc;

    //Example shooter hardware
    public CANSparkMax leftShooterMotor;
    public CANSparkMax rightShooterMotor;

    public SparkMaxPIDController leftShooterPID;
    public SparkMaxPIDController rightShooterPID;

    public RelativeEncoder leftShooterEnc;
    public RelativeEncoder rightShooterEnc;

    //Example elevator hardware
    public CANSparkMax rightElevatorMotor;
    public CANSparkMax leftElevatorMotor;

    public SparkMaxPIDController leftElevatorPID;
    public SparkMaxPIDController rightElevatorPID;

    public RelativeEncoder leftElevatorEnc;
    public RelativeEncoder rightElevatorEnc;
  

   


    public Hardware() {
        //Drivebase hardware
        leftDrive1 = new CANSparkMax(0, MotorType.kBrushless);
        leftDrive2 = new CANSparkMax(1, MotorType.kBrushless);
        rightDrive1 = new CANSparkMax(2, MotorType.kBrushless);
        rightDrive2 = new CANSparkMax(3, MotorType.kBrushless);

        leftDrive1Enc = leftDrive1.getEncoder();
        leftDrive2Enc = leftDrive2.getEncoder();
        rightDrive1Enc = rightDrive1.getEncoder();
        rightDrive2Enc = rightDrive2.getEncoder();

        leftDriveMotors = new SparkMAXMotorGroup("Left Drive Motor Group", leftDrive1, leftDrive2);
        rightDriveMotors = new SparkMAXMotorGroup("Right Drive Motor Group", rightDrive1, rightDrive2);

        leftDriveMotors.setIdleMode(IdleMode.kBrake);
        rightDriveMotors.setIdleMode(IdleMode.kBrake);

        //Claw hardware
        firstClawStage = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        secondClawStage = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

        //Simple arm hardware
        leftArm = new CANSparkMax(4, MotorType.kBrushless);
        rightArm = new CANSparkMax(5, MotorType.kBrushless);
        transArm = new CANSparkMax(6, MotorType.kBrushless);

        leftArmPIDConfig = new SparkMaxPIDConfig("Left Arm PID Controller", leftArm.getPIDController(), 0, 0, 0, 0);
        rightArmPIDConfig = new SparkMaxPIDConfig("Right Arm PID Controller", rightArm.getPIDController(), 0, 0, 0, 0);
        transArmPIDConfig = new SparkMaxPIDConfig("Trans Arm PID Controller", transArm.getPIDController(), 0, 0, 0, 0);

        leftArmPID = leftArmPIDConfig.getConfPIDController();
        rightArmPID = rightArmPIDConfig.getConfPIDController();
        transArmPID = transArmPIDConfig.getConfPIDController();
        
        leftArmEnc = leftArm.getEncoder();
        rightArmEnc = rightArm.getEncoder();
        transArmEnc = transArm.getEncoder();

        //Simple elevator hardware
        leftElevatorMotor = new CANSparkMax(6, MotorType.kBrushless);
        rightElevatorMotor = new CANSparkMax(7, MotorType.kBrushless);

        leftArmPIDConfig = new SparkMaxPIDConfig("Left Elevator Motor PID Controller", leftArm.getPIDController(), 0, 0, 0, 0);
        rightArmPIDConfig = new SparkMaxPIDConfig("Right Elevator Motor PID Controller", rightArm.getPIDController(), 0, 0, 0, 0);

        leftArmPID = leftArmPIDConfig.getConfPIDController();
        rightArmPID = rightArmPIDConfig.getConfPIDController();
        transArmPID = transArmPIDConfig.getConfPIDController();
        
        leftArmEnc = leftArm.getEncoder();
        rightArmEnc = rightArm.getEncoder();
        transArmEnc = rightArm.getEncoder();


    }


    



}
