package frc.robot.Subsystems;
//imports resources
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ExampleClaw extends SubsystemBase {
   // defines variables for solenoids
    private Solenoid firstStage;
    private Solenoid secondStage;
    //defines variables for stages
    private boolean firstStateStageStatus;
    private boolean secondStateStageStatus;


    public ExampleClaw(){
        // instantiates solenoids 
        firstStage = Robot.hardware.firstClawStage;
        secondStage = Robot.hardware.secondClawStage;
    }

    public void setSolenoidStatuses(String mode){
        switch(mode){
            case "HALF_COMPRESSION":
            //Boolean to set the solenoid to half compressed
                firstStateStageStatus = true;
                secondStateStageStatus = false;

            break;
            case "FULL_COMPRESSION":
            //Boolean to set the solenoid to fully compressed
                firstStateStageStatus = true;
                secondStateStageStatus = true;
            break;
            case "NO_COMPRESSION":
            //Boolean to set the solenoid to not compressed
                firstStateStageStatus = false;
                secondStateStageStatus = false;
            break;

        }
        //sets the solenoids to the compression status from the method
        firstStage.set(firstStateStageStatus);
        secondStage.set(secondStateStageStatus);
        
    }

    public boolean[] getSolenoidStatuses(){
        //a method to get the current solenoid statuses
        boolean stageStatuses[] = new boolean[2];
        stageStatuses[0]= firstStage.get();
        stageStatuses[1]= firstStage.get();

        return stageStatuses;

    }


    
       






}
