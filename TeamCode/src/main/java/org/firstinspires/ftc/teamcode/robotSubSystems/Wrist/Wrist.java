package org.firstinspires.ftc.teamcode.robotSubSystems.Wrist;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    public static final double POS_TRANSFER_LEFT = 0.03;
    public static final double POS_TRANSFER_RIGHT = 0.85;
    public static final double POS_INTAKE_LEFT = 0.8;
    public static final double POS_INTAKE_RIGHT = 0.2;

    public static final double POS_DEPLETE_LEFT = 0.55;
    public static final double POS_DEPLETE_RIGHT = 0.33;

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