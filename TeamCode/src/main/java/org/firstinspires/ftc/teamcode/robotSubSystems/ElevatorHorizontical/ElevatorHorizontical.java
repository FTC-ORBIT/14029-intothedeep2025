package org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.PID;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalConstants;

public class ElevatorHorizontical {
    private static DcMotor elevatorMotor;
    private static PID pid = new PID(ElevatorHorizonticalConstants.Kp,ElevatorHorizonticalConstants.Ki,ElevatorHorizonticalConstants.Kd,ElevatorHorizonticalConstants.Kf,ElevatorHorizonticalConstants.iZone,ElevatorHorizonticalConstants.maxSpeed,ElevatorHorizonticalConstants.minSpeed);

    private static float wantedPos;
    private static int resetVal = 0;

    public static void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class,"horElevator");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void opreate(ElevatorHorizonticalState elevatorHorizonticalState, float rightJoyStick){
                switch(elevatorHorizonticalState){
                    case OPEN:
                        wantedPos = ElevatorHorizonticalConstants.openPos;
                        break;
                    case CLOSE:
                        wantedPos = ElevatorHorizonticalConstants.closedPos;
                        break;
                    case ALMOST:
                        wantedPos = ElevatorHorizonticalConstants.almostPos;
                        break;
                    case HALF:
                        wantedPos = ElevatorHorizonticalConstants.halfpos;
                        break;
                    case OVERRIDE:
                        wantedPos += ElevatorHorizonticalConstants.overrideFactor * -rightJoyStick;
                        break;
                }
                pid.setWanted(wantedPos);
                elevatorMotor.setPower(pid.update(getElevatorPos()));
    }
    public static boolean inPos(){
        return wantedPos - ElevatorVerticalConstants.posTolerance
                < getElevatorPos() &&
                wantedPos + ElevatorVerticalConstants.posTolerance > getElevatorPos();
    }
    public static double getElevatorPos() {return elevatorMotor.getCurrentPosition() - resetVal;}
    public static void resetEncoder() {resetVal = elevatorMotor.getCurrentPosition();}
}
