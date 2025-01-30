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
                leftWristServo.setPosition(0.04);
                rightWristServo.setPosition(0.96);
                break;
            case INTAKE:
                leftWristServo.setPosition(0.68);
                rightWristServo.setPosition(0.28);
                break;
            case DEPLETE:
                leftWristServo.setPosition(0.67);
                rightWristServo.setPosition(0.35);
        }
    }
}
