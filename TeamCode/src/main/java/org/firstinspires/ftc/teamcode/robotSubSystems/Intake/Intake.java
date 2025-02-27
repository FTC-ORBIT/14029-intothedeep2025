package org.firstinspires.ftc.teamcode.robotSubSystems.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public static Servo rightIntakeServo;
    public static Servo leftIntakeServo;

    public static Servo intakeBar;
    public static Servo kicker;


    public static void init(HardwareMap hardwareMap) {
        rightIntakeServo = hardwareMap.servo.get("rightIntakeServo");
        leftIntakeServo = hardwareMap.servo.get("leftIntakeServo");
        intakeBar = hardwareMap.servo.get("intakeBar");
        kicker = hardwareMap.servo.get("kicker");
    }

    public static void operate(IntakeState state) {
        switch (state) {
            case IN:
                leftIntakeServo.setPosition(-1);
                rightIntakeServo.setPosition(1);
                intakeBar.setPosition(1);
                break;
            case OFF:
                leftIntakeServo.setPosition(0.5);
                rightIntakeServo.setPosition(0.5);
                intakeBar.setPosition(0.5);
                break;
            case OUT:
                leftIntakeServo.setPosition(1);
                rightIntakeServo.setPosition(-1);
                intakeBar.setPosition(-1);
        }
    }
    public  static void kicker(boolean button) {
        if (button) {kicker.setPosition(0.55);} else {kicker.setPosition(0);}

    }
}