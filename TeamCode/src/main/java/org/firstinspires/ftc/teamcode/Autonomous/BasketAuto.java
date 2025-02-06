package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;

@Autonomous
public class BasketAuto extends LinearOpMode {
    final double robotCenterToArm = 10;
    Pose2d redBasket = RotatedPose2d.rotate90deg(new Pose2d(-64 + robotCenterToArm/Math.sqrt(2), -64 + robotCenterToArm/Math.sqrt(2),Math.toRadians(225)));
    Pose2d redGamePiece1 = RotatedPose2d.rotate90deg(new Pose2d(68 - robotCenterToArm, -26, Math.toRadians(0)));
    Pose2d redGamePiece2 = RotatedPose2d.rotate90deg(new Pose2d(58 - robotCenterToArm, -26, Math.toRadians(0)));
    Pose2d redGamePiece3 = RotatedPose2d.rotate90deg(new Pose2d(48 - robotCenterToArm, -26, Math.toRadians(0)));
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(new Pose2d(0,0,0))
                .splineToLinearHeading(redBasket, Math.toRadians(22.5-180-90))
                .strafeToLinearHeading(redGamePiece3.position,redGamePiece3.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .strafeToLinearHeading(redGamePiece2.position,redGamePiece2.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .strafeToLinearHeading(redGamePiece1.position,redGamePiece1.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading);
        waitForStart();

        Actions.runBlocking(actionBuilder.build());
    }
}
