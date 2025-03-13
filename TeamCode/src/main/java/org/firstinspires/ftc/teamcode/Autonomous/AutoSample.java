package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizontical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizonticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.Wrist;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.WristState;


@Autonomous(name = "AutoSample")
public class AutoSample extends LinearOpMode {
    public boolean isUp = true;
    public IntakeState robotIntakeState = IntakeState.OFF;
    WristState wristState = WristState.TRANSFER;

    ElevatorVerticalState lastelevatorVerticalState = ElevatorVerticalState.OFF;

    Pose2d redBasket = new Pose2d(-3,25 ,Math.toRadians(-45));
    Pose2d sample1 = new Pose2d(-13, 27,Math.toRadians(-13));
    Pose2d sample2 = new Pose2d(-13, 18,Math.toRadians(0));
    Pose2d sample3 = new Pose2d(-14, 23,Math.toRadians(22));
    Pose2d startPos = new Pose2d(0, 0,Math.toRadians(0));

    public ElevatorVerticalState robotVerticalElevatorState = ElevatorVerticalState.OFF;
    public ElevatorHorizonticalState robotHorizonticalElevatorState = ElevatorHorizonticalState.OFF;
    final double robotCenterToArm = 10;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        ElevatorVertical.init(hardwareMap);
        Arm.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        ElevatorHorizontical.init(hardwareMap);


        TrajectoryActionBuilder startToBasket = drive.actionBuilder(startPos)
                .strafeToLinearHeading(redBasket.position,startPos.heading)
                .turnTo(redBasket.heading);
        TrajectoryActionBuilder basketToSample1 = drive.actionBuilder(redBasket)
                .turnTo(sample1.heading)
                .strafeToLinearHeading(sample1.position,sample1.heading);
        TrajectoryActionBuilder basketToSample2 = drive.actionBuilder(redBasket)
                .turnTo(sample2.heading)
                .strafeToLinearHeading(sample2.position,sample2.heading);
        TrajectoryActionBuilder basketToSample3 = drive.actionBuilder(redBasket)
                .turnTo(sample3.heading)
                .strafeToLinearHeading(sample3.position,sample3.heading);
        TrajectoryActionBuilder sample1ToBasket = drive.actionBuilder(sample1)
                .turnTo(redBasket.heading)
                .strafeToLinearHeading(new Pose2d(-11,16.5 ,Math.toRadians(-45)).position,redBasket.heading);
        TrajectoryActionBuilder sample2ToBasket = drive.actionBuilder(sample2)
                .turnTo(redBasket.heading)
                .strafeToLinearHeading(new Pose2d(-3,1 ,Math.toRadians(-45)).position,redBasket.heading);
        TrajectoryActionBuilder sample3ToBasket = drive.actionBuilder(sample3)
                .turnTo(redBasket.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading);


        waitForStart();
//        Actions.runBlocking(actionBuilder.build());
        //Actions.runBlocking(new SequentialAction(sampleIntake() , sampleTransfer()));
//        Actions.runBlocking(depleteAction()));
//        Actions.runBlocking(new SequentialAction(sampleIntake(),sampleTransfer(),sampleDeplete()));
        Actions.runBlocking(new ParallelAction(startToBasket.build(), elevatorDeplete()));
        Actions.runBlocking( new SequentialAction( basketToSample1.build(), sampleIntake()));
        Actions.runBlocking(sampleTransfer());
        Actions.runBlocking(new ParallelAction(elevatorDeplete(), sample1ToBasket.build()));
        Actions.runBlocking(basketToSample2.build());
        Actions.runBlocking(sampleIntake());
        Actions.runBlocking(sampleTransfer());
        Actions.runBlocking(new ParallelAction(sample2ToBasket.build(), elevatorDeplete()));
    }


    public Action elevatorDeplete() {
        return new SequentialAction(
                elevatorVericalByState(ElevatorVerticalState.DEPLETE, true),
                new ParallelAction(
                        elevatorVericalByState(ElevatorVerticalState.DEPLETE, false),
                        new SequentialAction(
                                armByState(ArmState.DEPLETE),new SleepAction(0.5), armByState(ArmState.INTAKE), wristByState(WristState.DEPLETE) , elevatorVericalByState(ElevatorVerticalState.INTAKE, true))
                ));
    }

    public Action sampleIntake(){
       return new SequentialAction(
                new ParallelAction(
                        elevatorHorizontalByState(ElevatorHorizonticalState.HALF),
                        wristAction(WristState.INTAKE),
                        intakeByState(IntakeState.IN)
                ),new SleepAction(0.9)
        );
    }

    public Action sampleTransfer(){

        return new ParallelAction(
                        elevatorVericalByState(ElevatorVerticalState.INTAKE, false) ,
                        armByState(ArmState.INTAKE),
                        new SequentialAction(
                            new ParallelAction(
                                wristAction(WristState.TRANSFER)
                                ,intakeByState(IntakeState.OFF)
                            )
                            ,new SequentialAction(elevatorHorizontalByState(ElevatorHorizonticalState.CLOSE)
                            ,new SequentialAction(intakeByState(IntakeState.OUT))
                            ,new SequentialAction(new SleepAction(0.2), intakeByState(IntakeState.OFF))
                            ,elevatorVericalByState(ElevatorVerticalState.OFF, true)
                        )

        ));
    }
    public Action ArmDeplete() {
        return new SequentialAction(
                armByState(ArmState.DEPLETE), armByState(ArmState.INTAKE), wristByState(WristState.DEPLETE) , elevatorVericalByState(ElevatorVerticalState.INTAKE, true));
    }
    public Action elevatorVericalByState(final ElevatorVerticalState elevatorVerticalState, boolean stopAfterAction){
        return new Action() {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    lastelevatorVerticalState = elevatorVerticalState;
                    initialized = true;
                }
                ElevatorVertical.operate(elevatorVerticalState,0,0);
                telemetryPacket.put("elevatorVerticalState",elevatorVerticalState);
                telemetryPacket.put("lastElevatorVerticalState",lastelevatorVerticalState);
                telemetryPacket.put("elevator pos", ElevatorVertical.getElevatorPos());
                telemetryPacket.put("wantedPos", ElevatorVertical.getWantedPos());
                telemetryPacket.put("inPos",ElevatorVertical.inPos());
                if (ElevatorVertical.getElevatorPos() > 200 && ElevatorVertical.getElevatorPos() < 2250) {
                    Arm.operate(ArmState.HALF);
                }
                if (elevatorVerticalState ==ElevatorVerticalState.OFF) {
                    return false;
                }
                if (stopAfterAction) {
                    return !ElevatorVertical.inPos();
                }else{
                    return lastelevatorVerticalState==elevatorVerticalState;
                }
            }
        };
    }

    public Action elevatorHorizontalByState(final ElevatorHorizonticalState elevatorHorizonticalState){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElevatorHorizontical.opreate(elevatorHorizonticalState,0, false);
                telemetryPacket.put("elevatorHorizontalState",elevatorHorizonticalState);
                telemetryPacket.put("elevatorHpos", ElevatorHorizontical.getElevatorPos());
                telemetryPacket.put("wantedHPos", ElevatorHorizontical.getWantedPos());
                telemetryPacket.put("inPosH",ElevatorHorizontical.inPos());
                return !ElevatorHorizontical.inPos();
            }
        };
    }


    public Action armByState(ArmState armState){
        Action armAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.operate(armState);
                telemetryPacket.put("arm position",Arm.armServo.getPosition());
                return false;
            }

        };
        return new SequentialAction(armAction,new SleepAction(1));
    }
    public Action wristByState(final WristState wristState) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Wrist.operate(wristState);
                return false;
            }
        };



    }

//    public Action intakeByState(IntakeState intakeState){
//        return new Action() {
//            private boolean isInitialized = false;
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if (!isInitialized){
//                    robotIntakeState = intakeState;
//                    isInitialized = true;
//                }
//                Intake.operate(robotIntakeState);
//                return robotIntakeState != IntakeState.OFF;
//            }
//
//        };
//    }


    public Action intakeByState(IntakeState intakeState){
        Action servoIntakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.put("leftWrist",Intake.leftIntakeServo.getPosition());
                Intake.operate(intakeState);
                if (intakeState == IntakeState.IN) {
                    return Intake.rightIntakeServo.getPosition() < Intake.POS_IN_RIGHT_INTAKE;
                } else if (intakeState == IntakeState.OFF) {
                    return Intake.rightIntakeServo.getPosition() < Intake.POS_OFF_RIGHT_INTAKE;
                } else {
                    return Intake.rightIntakeServo.getPosition() < Intake.POS_OUT_RIGHT_INTAKE;
                }
            }
        };
        return new SequentialAction(servoIntakeAction , new SleepAction(1));

    }
    public Action wristAction(WristState wristState){
        Action servoWristAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                telemetryPacket.put("rightServoPos",Wrist.rightWristServo.getPosition());
                Wrist.operate(wristState);
                if (wristState == WristState.INTAKE) {
                    return (Wrist.rightWristServo.getPosition() < Wrist.POS_INTAKE_RIGHT);
                } else if (wristState == WristState.DEPLETE) {
                    return Wrist.rightWristServo.getPosition() < Wrist.POS_DEPLETE_RIGHT;
                } else {
                    return Wrist.rightWristServo.getPosition() < Wrist.POS_TRANSFER_RIGHT;
                }
            }
        };
        return new SequentialAction(servoWristAction , new SleepAction(1));

    }
    public Action intake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                elevatorHorizontalByState(ElevatorHorizonticalState.HALF);
                wristByState(WristState.INTAKE);
                intakeByState(IntakeState.IN);
                return Intake.leftIntakeServo.getPosition() <= Intake.POS_OUT_LEFT_INTAKE && Wrist.leftWristServo.getPosition() <= Wrist.POS_INTAKE_LEFT && !ElevatorHorizontical.inPos();
            }
        };
    }
//
    public Action driveAction(Pose2d start, Pose2d end, MecanumDrive drivetrain){
        TrajectoryActionBuilder drive = drivetrain.actionBuilder(start)
                .turnTo(end.heading)
                .strafeToLinearHeading(end.position,end.heading);
        return drive.build();

    }


};



