package org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.PID;

public class ElevatorHorizontical {
    public static DcMotor elevatorMotor;
    private static PID pidTeleOP = new PID(ElevatorHorizonticalConstants.KpTeleop,ElevatorHorizonticalConstants.Ki,ElevatorHorizonticalConstants.Kd,ElevatorHorizonticalConstants.Kf,ElevatorHorizonticalConstants.iZone,ElevatorHorizonticalConstants.maxSpeed,ElevatorHorizonticalConstants.minSpeed);

    private static PID pidAuto = new PID(ElevatorHorizonticalConstants.KpAuto,ElevatorHorizonticalConstants.Ki,ElevatorHorizonticalConstants.Kd,ElevatorHorizonticalConstants.Kf,ElevatorHorizonticalConstants.iZone,ElevatorHorizonticalConstants.maxSpeed,ElevatorHorizonticalConstants.minSpeed);

    private static int wantedPos;
    private static int resetVal = 0;

    public static void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class,"horElevator");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void opreate(ElevatorHorizonticalState elevatorHorizonticalState, float rightJoyStick, boolean isTeleop){
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
                        wantedPos += (int) ElevatorHorizonticalConstants.overrideFactor * -rightJoyStick;
                        break;
                }
                if (isTeleop) {
                    pidTeleOP.setWanted(wantedPos);
                    elevatorMotor.setPower(pidTeleOP.update(getElevatorPos()));
                } else {
                    pidAuto.setWanted(wantedPos);
                    elevatorMotor.setPower(pidAuto.update(getElevatorPos()));
                }
    }
    public static boolean inPos(){
        return wantedPos - ElevatorHorizonticalConstants.posTolerance
                < getElevatorPos() &&
                wantedPos + ElevatorHorizonticalConstants.posTolerance > getElevatorPos();
    }
    public static boolean atLeastPose(){
        return wantedPos + ElevatorHorizonticalConstants.posTolerance > getElevatorPos();
    }
    public static double getElevatorPos() {return elevatorMotor.getCurrentPosition() - resetVal;}
    public static void resetEncoder() {resetVal = elevatorMotor.getCurrentPosition();}
     public static int getWantedPos(){return wantedPos;}
}
