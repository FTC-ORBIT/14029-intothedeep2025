package org.firstinspires.ftc.teamcode.robotSubSystems.Wrist;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    public static Servo rightWristServo;
    public static Servo leftWristServo;


    public static void init(HardwareMap hardwareMap) {
        rightWristServo = hardwareMap.servo.get("rightWristServo");
        leftWristServo = hardwareMap.servo.get("leftWristServo");
    }

    public static void operate(WristState state) {
        switch (state) {
            case TRANSFER:
                //leftWristServo.setPosition(0.03);//0.025
                rightWristServo.setPosition(0.9);//0.95
                break;
            case INTAKE:
                //leftWristServo.setPosition(0.8);
                rightWristServo.setPosition(0.2);
                break;
            case DEPLETE:
                leftWristServo.setPosition(0.55);//0.55
                rightWristServo.setPosition(0.425);//0.425
        }
    }
}