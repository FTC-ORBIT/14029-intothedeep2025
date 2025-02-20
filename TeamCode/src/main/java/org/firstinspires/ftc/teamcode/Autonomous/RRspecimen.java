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
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;

@Autonomous(name = "auto")
public class RRspecimen extends LinearOpMode {
    boolean isUp= true;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        ElevatorVertical.init(hardwareMap);
        TrajectoryActionBuilder firstBuilder= drive.actionBuilder(new Pose2d(0,0,0)).
                strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(0,-30,0)).position,0);
        TrajectoryActionBuilder secondBuilder= drive.actionBuilder(new Pose2d(0,-30,0)).
                strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-45,-12,0)).position,0);
        waitForStart();

        Actions.runBlocking(new ParallelAction(
                        specimen(),firstBuilder.build(),
                        new SequentialAction( new SleepAction(2),stopSpecimen())));
        Actions.runBlocking(intake());
        Actions.runBlocking(secondBuilder.build());
    }

    public Action specimen(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElevatorVertical.operate(ElevatorVerticalState.PUTSPECIMEN, 0,0);
                return ElevatorVertical.getElevatorPos() <= ElevatorVerticalConstants.DepletePos -5 && isUp;
            }

        };
    }
    public Action intake(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElevatorVertical.operate(ElevatorVerticalState.INTAKE, 0,0);
                return ElevatorVertical.getElevatorPos() <= ElevatorVerticalConstants.IntakePos -5;
            }

        };
    }


    public Action stopSpecimen(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                isUp=false;
                return false;
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

}
