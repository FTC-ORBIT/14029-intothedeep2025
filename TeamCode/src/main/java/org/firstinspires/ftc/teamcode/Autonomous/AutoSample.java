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
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalConstants;
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

    Pose2d redBasket = new Pose2d(-12,117 ,Math.toRadians(-45));
    Pose2d sample1 = new Pose2d(-22, 109,Math.toRadians(0));
    Pose2d sample2 = new Pose2d(-22, 119,Math.toRadians(0));
    Pose2d sample3 = new Pose2d(-23, 113,Math.toRadians(38));
    Pose2d startPos = new Pose2d(98, 0,Math.toRadians(0));

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


        waitForStart();
//        Actions.runBlocking(actionBuilder.build());
//        Actions.runBlocking(new ParallelAction(
//                elevatorVericalByState(ElevatorVerticalState.DEPLETE),
//                new SequentialAction(
//                    armHalf(),new SleepAction(2),armDeplete(),new SleepAction(4),stopBasket(), elevatorVericalByState(ElevatorVerticalState.INTAKE))
//                )
//
//        );
//        Actions.runBlocking(new SequentialAction(wristAction(WristState.INTAKE) ,new SleepAction(1)));
//        Actions.runBlocking(new SequentialAction(intakeByState(IntakeState.IN) , new SleepAction(1)));
//        Actions.runBlocking(new SequentialAction(wristAction(WristState.TRANSFER)));

        Actions.runBlocking(new SequentialAction(sampleIntake() , sampleTransfer()));
//        Actions.runBlocking(elevatorVericalByState(ElevatorVerticalState.SPECIMEN));

    }


    public Action sampleIntake(){
       return new SequentialAction(
                new ParallelAction(
                        wristAction(WristState.INTAKE)
                        ,intakeByState(IntakeState.IN)
                ),new SleepAction(2)
        );
    }

    public Action sampleTransfer(){

        return new ParallelAction(
                new SequentialAction(
                    new ParallelAction(
                        wristAction(WristState.TRANSFER)
                        ,intakeByState(IntakeState.OFF)


                )
                ,intakeByState(IntakeState.OUT)
                ,new SleepAction(1)
                )
                ,elevatorVericalByState(ElevatorVerticalState.SPECIMEN)
        );
    }
    public Action sampleBasket(){
        return new SequentialAction(
                new ParallelAction(
                        elevatorVericalByState(ElevatorVerticalState.DEPLETE)
                        ,armHalf()
                )
                ,new ParallelAction(
                        elevatorVericalByState(ElevatorVerticalState.DEPLETE)
                    ,armDeplete()
                )
        );
    }
    public Action highBasket(){
        isUp=true;
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElevatorVertical.operate(ElevatorVerticalState.DEPLETE, 0,0);
                return ElevatorVertical.getElevatorPos() <= ElevatorVerticalConstants.DepletePos -5 && isUp;
            }

        };
    }

    public Action elevatorVerticalSampleTransfer(){
        isUp=true;
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElevatorVertical.operate(ElevatorVerticalState.SPECIMEN, 0,0);
                return ElevatorVertical.getElevatorPos() <= ElevatorVerticalConstants.SpecimenPos -5 && isUp;
            }

        };
    }


    public Action stopBasket(){
       return new Action() {
           @Override
           public boolean run(@NonNull TelemetryPacket telemetryPacket) {
               isUp=false;
               return false;
           }
       };

    }

    public Action elevatorVericalByState(final ElevatorVerticalState elevatorVerticalState){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.put("elevator pos", ElevatorVertical.getElevatorPos());
                ElevatorVertical.operate(elevatorVerticalState,0,0);
                return !ElevatorVertical.inPos();
            }
        };
    }

    public Action elevatorHorizontalByState(final ElevatorHorizonticalState elevatorHorizonticalState){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElevatorHorizontical.opreate(elevatorHorizonticalState,0);
                return !ElevatorHorizontical.inPos();
            }
        };
    }


    public Action armDeplete(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.operate(ArmState.DEPLETE);
                telemetryPacket.put("arm position",Arm.armServo.getPosition());
                return Arm.armServo.getPosition() <= 0.74;
            }

        };
    }
    public Action armHalf(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ElevatorVertical.getElevatorPos() > 200) {
                    Arm.operate(ArmState.HALF);
                }
                return Arm.armServo.getPosition() <= 0.36;
            }

        };
    }
    public Action wristByState(final WristState wristState) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Wrist.operate(wristState);
                return Wrist.leftWristServo.getPosition() <= 0.69;
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
                    return (Intake.rightIntakeServo.getPosition() < Intake.POS_IN_RIGHT_INTAKE);
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
    public Action transferSample(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wristByState(WristState.TRANSFER);
                elevatorVericalByState(ElevatorVerticalState.INTAKE);
                sleep(2002);
                intakeByState(IntakeState.OUT);

                return Intake.leftIntakeServo.getPosition() <= Intake.POS_OUT_LEFT_INTAKE && Wrist.rightWristServo.getPosition() <= Wrist.POS_DEPLETE_RIGHT && !ElevatorVertical.inPos();
            }
        };
    }

};



