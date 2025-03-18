package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizontical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.Wrist;

@Autonomous(name = "autoSpecimen")
public class AutoSpecimen extends LinearOpMode {
    ElevatorVerticalState lastelevatorVerticalState = ElevatorVerticalState.OFF;
    boolean isUp= true;boolean isUp1= true;



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,63,0));
        ElevatorVertical.init(hardwareMap);
        Arm.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        ElevatorHorizontical.init(hardwareMap);

        VelConstraint velConstraint = new TranslationalVelConstraint(100);
        AccelConstraint accelConstraint = new ProfileAccelConstraint(-52,87);

        TrajectoryActionBuilder firstBuilder = drive.actionBuilder(new Pose2d(0, 63, 0)).
                strafeTo(new Pose2d(32, 63, 0).position, velConstraint, accelConstraint);
        TrajectoryActionBuilder secondBuilder = drive.actionBuilder(new Pose2d(32, 63, 0)).
                strafeToLinearHeading(new Pose2d(10, 90, 0).position, 0)
                .turnTo(Math.toRadians(180));
        TrajectoryActionBuilder thirdBuilder = drive.actionBuilder(new Pose2d(10, 90, Math.toRadians(180))).
                strafeToLinearHeading(new Vector2d(0,105),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-5,105),Math.toRadians(180));
        TrajectoryActionBuilder fourthBuilder = drive.actionBuilder(new Pose2d(-5, 105, Math.toRadians(180))).
                strafeTo(new Pose2d(15, 63, Math.toRadians(180)).position)
                .turnTo(Math.toRadians(0))
                .strafeTo(new Pose2d(15,63+5,0).position)
                .strafeTo(new Pose2d(32,63+5,0).position);

        TrajectoryActionBuilder fifthBuilder = drive.actionBuilder(new Pose2d(32, 63, 0))
                        .strafeTo(new Vector2d(32-17,63+40), velConstraint, accelConstraint)
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(32+19, 63+40), velConstraint, accelConstraint)
                .strafeTo(new Vector2d(32+19, 63+40+10), velConstraint, accelConstraint)
                .strafeTo(new Vector2d(5, 63+40+10+8), velConstraint, accelConstraint)
                ;

        TrajectoryActionBuilder spline = drive.actionBuilder(new Pose2d(32, 63, 0))
                .splineTo(new Vector2d(32-15, 63+40), Math.toRadians(-90));



        TrajectoryActionBuilder sixthBuilder = drive.actionBuilder(new Pose2d(5, 63+40+10+8, Math.toRadians(180)))
                .strafeTo(new Vector2d(5+8, 63+40+10+8-12), velConstraint, accelConstraint)
                .strafeTo(new Vector2d(-5, 63+40+10+8-12));
        TrajectoryActionBuilder seventhBuilder = drive.actionBuilder(new Pose2d(0, 63+40+10+8-12, Math.toRadians(180))).
                strafeToLinearHeading(new Vector2d(10,65),Math.toRadians(180), velConstraint, accelConstraint)
                .turnTo(0)
                .strafeTo(new Vector2d(35,70), velConstraint, accelConstraint);
        TrajectoryActionBuilder eighthBuilder = drive.actionBuilder(new Pose2d(32, 63, Math.toRadians(0))).
                strafeTo(new Vector2d(5+8,63+40+10+8-12), velConstraint, accelConstraint)
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(-5,63+40+10+8-12));
        TrajectoryActionBuilder ninthBuilder = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(180))).strafeTo(new Vector2d(0,10));//.turnTo(0);//.strafeTo(new Vector2d(-36,36)); // 11 , -36
        TrajectoryActionBuilder tenthBuilder = drive.actionBuilder(new Pose2d(0, 63+40+10+8-12, Math.toRadians(180))).
                strafeToLinearHeading(new Vector2d(15,65),Math.toRadians(180), velConstraint, accelConstraint).turnTo(0)
                .strafeTo(new Vector2d(45,70));


        waitForStart();

            Actions.runBlocking(
                    //first move and putting the first specimen
                    new ParallelAction(
                            elevatorVericalByState(ElevatorVerticalState.PUTSPECIMEN, false)
                            ,firstBuilder.build()
                            , new SequentialAction( new SleepAction(1.5),elevatorVericalByState(ElevatorVerticalState.SPECIMEN, true))
                    )
            );
            Actions.runBlocking(
                new SequentialAction(
                        // going to move the sample from the field to the human player area
                        fifthBuilder.build(),
                        new ParallelAction(
                                //pick up the second specimen from the human player
                                sixthBuilder.build(),
                                elevatorVericalByState(ElevatorVerticalState.SPECIMEN, false),
                            new SequentialAction(
                                new SleepAction(1.5),elevatorVericalByState(ElevatorVerticalState.PUTSPECIMEN, true)
                            )
                        )
                )
            );
            Actions.runBlocking(
                    new ParallelAction(
                            //put the specimen on the bar
                            seventhBuilder.build(),
                            elevatorVericalByState(ElevatorVerticalState.PUTSPECIMEN, false),
                            new SequentialAction(new SleepAction(5), elevatorVericalByState(ElevatorVerticalState.SPECIMEN, true))
                    )
            );
            Actions.runBlocking(
                    //picking up the next specimen from the human player
                    new ParallelAction(
                            eighthBuilder.build(),
                            elevatorVericalByState(ElevatorVerticalState.SPECIMEN, false),
                            new SequentialAction(
                                    new SleepAction(4.5), elevatorVericalByState(ElevatorVerticalState.PUTSPECIMEN, true)
                            )
                    )
            );
            Actions.runBlocking(
                    new ParallelAction(
                            tenthBuilder.build(),
                            elevatorVericalByState(ElevatorVerticalState.PUTSPECIMEN,false),
                            new SequentialAction(
                                    new SleepAction(5),elevatorVericalByState(ElevatorVerticalState.INTAKE, true)
                            )
                    )
            );

//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                            sixthBuilder.build()
//                            ,elevatorVericalByState(ElevatorVerticalState.SPECIMEN,true),
//                                new SleepAction(2)
//                        ),printElevatorCurrent()
//                        ,new ParallelAction(
//                            seventhBuilder.build()
//                            ,new SequentialAction(elevatorVericalByState(ElevatorVerticalState.ZERO,true),elevetorVerticalReset(),elevetorVerticalReset() ,elevatorVericalByState(ElevatorVerticalState.PUTSPECIMEN,true))
//                        ),new ParallelAction(
//                                eighthBuilder.build()
//                                ,elevatorVericalByState(ElevatorVerticalState.SPECIMEN,true)
//                        ),new ParallelAction(
//                                seventhBuilder.build()
//                                ,elevatorVericalByState(ElevatorVerticalState.PUTSPECIMEN,true)
//                        )
//
//
//                )
//
//        );
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                secondBuilder.build()
//                                , elevatorVericalByState(ElevatorVerticalState.INTAKE,true)
//                        )
//                        ,new ParallelAction(
//                                thirdBuilder.build()
//                                , elevatorVericalByState(ElevatorVerticalState.SPECIMEN,true)
//                        )
//                )
//        );
//        Actions.runBlocking(
//                new ParallelAction(
//                        fourthBuilder.build(),
//                        elevatorVericalByState(ElevatorVerticalState.PUTSPECIMEN,true)
//                )
//        );
    }
    public Action elevetorVerticalReset(){
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElevatorVertical.resetEncoder();
                return false;
            }
        };
    }
    public Action elevatorVericalByState(final ElevatorVerticalState elevatorVerticalState, boolean stopAfterAction) {
        return new Action() {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    lastelevatorVerticalState = elevatorVerticalState;
                    initialized = true;
                }
                ElevatorVertical.operate(elevatorVerticalState, 0, 0);
                telemetryPacket.put("elevatorVerticalState", elevatorVerticalState);
                telemetryPacket.put("lastElevatorVerticalState", lastelevatorVerticalState);
                telemetryPacket.put("elevator pos", ElevatorVertical.getElevatorPos());
                telemetryPacket.put("wantedPos", ElevatorVertical.getWantedPos());
                telemetryPacket.put("inPos", ElevatorVertical.inPos());
                if (elevatorVerticalState == ElevatorVerticalState.OFF) {
                    return false;
                }
                if (stopAfterAction) {
                    return !ElevatorVertical.inPos();
                } else {
                    return lastelevatorVerticalState == elevatorVerticalState;
                }
            }
        };
    }
    public Action printElevatorCurrent() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("elevatorCurrent", ElevatorVertical.getElevatorPos());
                return false;
            }
        };
    }


}
