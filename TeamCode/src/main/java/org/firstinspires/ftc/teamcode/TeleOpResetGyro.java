package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.AngleStorage;
import org.firstinspires.ftc.teamcode.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utils.GlobalData;
import org.firstinspires.ftc.teamcode.Utils.Vector;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizontical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizonticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.Wrist;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.WristState;

@TeleOp(name = "TeleOp - reset gyro")
public class TeleOpResetGyro extends OpMode {
    ElapsedTime elapsedTime = new ElapsedTime();
    boolean firstTimePressedleft_bumper;
    float left_bumperTimer = 0;

    boolean firstTimePressedA;
    float aTimer;
    IntakeState intakeState = IntakeState.OFF;
    WristState wristState = WristState.TRANSFER;
    ArmState armState = ArmState.INTAKE;
    ElevatorVerticalState elevatorState = ElevatorVerticalState.INTAKE;
    ElevatorHorizonticalState elevatorHorizonticalState = ElevatorHorizonticalState.ALMOST;
    @Override
    public void init() {
        Drivetrain.init(hardwareMap);
        Gyro.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        Arm.init(hardwareMap);
        ElevatorHorizontical.init(hardwareMap);
        ElevatorVertical.init(hardwareMap);
        elapsedTime.reset();
        Gyro.resetGyroForTeleop(AngleStorage.angle);
    }
    @Override
    public void loop() {
        GlobalData.currentTime = (float) elapsedTime.milliseconds();
        //vertical elevators -------------------------------------------------------------------------
        if (gamepad1.a) {elevatorState = ElevatorVerticalState.INTAKE; elevatorHorizonticalState = ElevatorHorizonticalState.CLOSE;}
        if (elevatorState != ElevatorVerticalState.INTAKE && ElevatorVertical.getElevatorPos() > 220) {armState = ArmState.HALF;}else {armState = ArmState.INTAKE;}
        if (gamepad1.b) {elevatorState = ElevatorVerticalState.SPECIMEN;}
        if (gamepad1.x) {elevatorState = ElevatorVerticalState.PUTSPECIMEN;}
        if (gamepad1.y) {elevatorHorizonticalState = ElevatorHorizonticalState.HALF; elevatorState = ElevatorVerticalState.DEPLETE; intakeState=IntakeState.OFF;}
        if (gamepad1.right_stick_y != 0) {elevatorState = elevatorState.MANUAL;}
        if (gamepad1.start || gamepad2.start) {ElevatorVertical.resetEncoder(); ElevatorHorizontical.resetEncoder();}
        //horizontal elevators ------------------------------------------------------------------------
        if (gamepad1.right_stick_x != 0) {elevatorHorizonticalState = ElevatorHorizonticalState.OVERRIDE;}
        if (gamepad1.dpad_down) {
            wristState = WristState.INTAKE; intakeState = IntakeState.IN;
            elevatorHorizonticalState = ElevatorHorizonticalState.OPEN;
        }
        //general----------------------------------------------------------------------------
        if (gamepad1.right_bumper) {elevatorHorizonticalState = ElevatorHorizonticalState.HALF; wristState = WristState.INTAKE; intakeState = IntakeState.IN;}
        if (gamepad1.left_bumper) {wristState = WristState.TRANSFER; intakeState = IntakeState.OFF; elevatorHorizonticalState = ElevatorHorizonticalState.CLOSE;}

        if (gamepad1.right_stick_button) {intakeState = IntakeState.OUT;}
        if (gamepad1.left_stick_button) {intakeState = IntakeState.OFF;}
        if (gamepad1.dpad_up) {armState = ArmState.DEPLETE;}
        if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad2.dpad_left || gamepad2.dpad_right) {wristState = WristState.DEPLETE; intakeState = IntakeState.OFF;}

        if (gamepad2.x) {intakeState = IntakeState.OUT;}
        if (gamepad2.a) {intakeState = IntakeState.OFF;}
        Intake.kicker(gamepad2.y);

        if (gamepad1.back) {Gyro.resetGyro();}

        //TODO: add encoder limiters for horizontal
        ElevatorHorizontical.opreate(elevatorHorizonticalState,gamepad1.right_stick_x, true);
        ElevatorVertical.operate( elevatorState, -gamepad1.right_stick_y, -gamepad2.right_stick_y);
        Intake.operate(intakeState);
        Arm.operate(armState);
        Wrist.operate(wristState);
        Drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y), -gamepad1.right_trigger + gamepad1.left_trigger);
        telemetry.addData("Gyro", Gyro.getAngle());
        telemetry.addData("horizontalPos", ElevatorHorizontical.getElevatorPos());
        telemetry.addData("verticalPos", ElevatorVertical.getElevatorPos());
        telemetry.addData("timer", GlobalData.currentTime);
        telemetry.addData("timer 2", GlobalData.currentTime - left_bumperTimer);
        telemetry.addData("bool", firstTimePressedleft_bumper);
        telemetry.addData("intakestate", intakeState);
    }
}

