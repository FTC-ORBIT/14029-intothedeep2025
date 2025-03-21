package org.firstinspires.ftc.teamcode.robotSubSystems.Wrist;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    public static final double POS_TRANSFER_LEFT = 0.02;
    public static final double POS_TRANSFER_RIGHT = 0.86;
    public static final double POS_INTAKE_LEFT = 0.679; // 0.677
    public static final double POS_INTAKE_RIGHT = 0.169     ; //0.18

    public static final double POS_DEPLETE_LEFT = 0.48;
    public static final double POS_DEPLETE_RIGHT = 0.4;

    public static Servo rightWristServo;
    public static Servo leftWristServo;


    public static void init(HardwareMap hardwareMap) {
        rightWristServo = hardwareMap.servo.get("rightWristServo");
        leftWristServo = hardwareMap.servo.get("leftWristServo");
    }

    public static void operate(WristState state) {
        switch (state) {
            case TRANSFER:
                leftWristServo.setPosition(POS_TRANSFER_LEFT);//0.025
                rightWristServo.setPosition(POS_TRANSFER_RIGHT);//0.95
                break;
            case INTAKE:
                leftWristServo.setPosition(POS_INTAKE_LEFT);
                rightWristServo.setPosition(POS_INTAKE_RIGHT);//
                break;
            case DEPLETE:
                leftWristServo.setPosition(POS_DEPLETE_LEFT);//0.55
                rightWristServo.setPosition(POS_DEPLETE_RIGHT);//0.425
        }
    }
}