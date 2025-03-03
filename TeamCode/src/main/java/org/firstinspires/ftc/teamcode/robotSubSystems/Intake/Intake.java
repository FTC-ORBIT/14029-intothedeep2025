package org.firstinspires.ftc.teamcode.robotSubSystems.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public static Servo rightIntakeServo;
    public static Servo leftIntakeServo;

    public static Servo intakeBar;
    public static Servo kicker;


    public static final double POS_IN_RIGHT_INTAKE = 1;
    public static final double POS_IN_LEFT_INTAKE = -1;
    public static final double POS_IN_INTAKE_BAR = 1;
    public static final double POS_OFF_RIGHT_INTAKE = 0.5;
    public static final double POS_OFF_LEFT_INTAKE = 0.5;
    public static final double POS_OFF_INTAKE_BAR = 0.5;
    public static final double POS_OUT_LEFT_INTAKE = 1;
    public static final double POS_OUT_RIGHT_INTAKE= -1;
    public static final double POS_OUT_INTAKE_BAR = -1;








    public static void init(HardwareMap hardwareMap) {
        rightIntakeServo = hardwareMap.servo.get("right");
        leftIntakeServo = hardwareMap.servo.get("left");
        intakeBar = hardwareMap.servo.get("intakeBar");
        kicker = hardwareMap.servo.get("kicker");
    }

    public static void operate(IntakeState state) {
        switch (state) {
            case IN:
                intakeBar.setPosition(POS_IN_INTAKE_BAR);
                leftIntakeServo.setPosition(POS_IN_LEFT_INTAKE);
                rightIntakeServo.setPosition(POS_IN_RIGHT_INTAKE);

                break;
            case OFF:
                leftIntakeServo.setPosition(POS_OFF_LEFT_INTAKE);
                rightIntakeServo.setPosition(POS_OFF_RIGHT_INTAKE);
                intakeBar.setPosition(POS_OFF_INTAKE_BAR);
                break;
            case OUT:
                leftIntakeServo.setPosition(POS_OUT_LEFT_INTAKE);
                rightIntakeServo.setPosition(POS_OUT_RIGHT_INTAKE);
                intakeBar.setPosition(POS_OUT_INTAKE_BAR);
        }
    }
    public  static void kicker(boolean button) {
        if (button) {kicker.setPosition(0.55);} else {kicker.setPosition(0);}

    }
}