package org.firstinspires.ftc.teamcode.robotSubSystems.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class Drivetrain {

    private static final DcMotor[] dtMotors = new DcMotor[4];

    public static void init(HardwareMap hardwareMap) {
        dtMotors[0] = hardwareMap.get(DcMotor.class, "lf");
        dtMotors[1] = hardwareMap.get(DcMotor.class, "lb");
        dtMotors[2] = hardwareMap.get(DcMotor.class, "rf");
        dtMotors[3] = hardwareMap.get(DcMotor.class, "rb");

        for (final DcMotor dtMotors : dtMotors) {
            dtMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //todo: set reverse directions
        dtMotors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        dtMotors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        dtMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        dtMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        dtMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoders();
    }

    public static void operate(Vector vector, double rotation) {
        drive(
                slowedVec(
                        fieldCentric(vector), 0
                         //Elevator.getElevatorPos()
                        , 2235,
                          0.4
                ),
                rotation
        );
    }

    private static Vector slowedVec(Vector vector, double current, double highest, double speed) {
        //0.32
        return vector.scale(Math.max(speed, (highest - current) / highest));
    }

    private static Vector fieldCentric(Vector gamepad) {
        gamepad = gamepad.rotate(Math.toRadians(Angle.wrapAnglePlusMinus_180(-Gyro.getAngle())));
        return gamepad;
    }

    public static void drive(Vector vector, double rotation) {
        dtMotors[0].setPower(Math.signum(vector.y + vector.x + rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y + vector.x + rotation))));
        dtMotors[1].setPower(Math.signum(vector.y - vector.x + rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y - vector.x + rotation))));
        dtMotors[2].setPower(Math.signum(vector.y - vector.x - rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y - vector.x - rotation))));
        dtMotors[3].setPower(Math.signum(vector.y + vector.x - rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y + vector.x - rotation))));
    }
    public static void driveByTime(double power) {
        dtMotors[0].setPower(-power);
        dtMotors[1].setPower(-power);
        dtMotors[2].setPower(-power);
        dtMotors[3].setPower(-power);
    }


    public static double getEncoderPos() {
        return dtMotors[3].getCurrentPosition();
    }

    public static void resetEncoders() {
        dtMotors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dtMotors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public static void breakMotors() {
        for (DcMotor motor : dtMotors) {
            motor.setPower(0);
        }
    }


    public static double cmToTicks(double cm) {
        return cm * 65.19;
    }



    public static void testMotors(Gamepad gamepad){
       if (gamepad.a) dtMotors[0].setPower(0.2); // lf
       else dtMotors[0].setPower(0);
       if (gamepad.b) dtMotors[1].setPower(0.2); // lb
       else dtMotors[1].setPower(0);
       if (gamepad.x) dtMotors[2].setPower(0.2); // rf
       else dtMotors[2].setPower(0);
       if (gamepad.y) dtMotors[3].setPower(0.2); // rb
       else dtMotors[3].setPower(0);
    }

    public static void testDeadWheels(Telemetry telemetry){
        telemetry.addData("par",dtMotors[0].getCurrentPosition());
        telemetry.addData("prep",dtMotors[3]);
    }


    /*
    public static void moveRobot(Pose2D wanted, Telemetry telemetry) {
//        wanted.vector.rotate(Angle.wrapAngle0_360(Gyro.getAngle()));


        final double lfWanted = wanted.getY() + wanted.getX();
        final double rfWanted = wanted.getY() - wanted.getX();

        double lfCurrent = 0;
        double rfCurrent = 0;


//        turn robot pid

        turnRobotPID.setWanted(wanted.getAngle());

        double rotationPower = turnRobotPID.update(-Gyro.getAngle());

        telemetry.addData("power", lfPower);
        telemetry.addData("posY", PoseTracker.getPose().getY());
        telemetry.addData("posX", PoseTracker.getPose().getX());

        boolean isFinishedTurning = false;
        boolean isFinishedLf = false;
        boolean isFinishedRf = false;


//        if (wanted.getY() != 0 || wanted.getX() != 0){
//            dtMotors[0].setPower(lfPower);
//            dtMotors[1].setPower(rfPower);
//            dtMotors[2].setPower(rfPower * 0.9);
//            dtMotors[3].setPower(lfPower * 0.9);

        if (wanted.getY() == 0 && wanted.getX() == 0) {
            rotationPower = Math.max(0.15, Math.abs(rotationPower)) * Math.signum(rotationPower);

            isFinishedLf = true;
            isFinishedRf = true;
            lfPower = 0;
            rfPower = 0;

            dtMotors[0].setPower(lfPower + rotationPower);
            dtMotors[1].setPower(rfPower + rotationPower);
            dtMotors[2].setPower(rfPower - rotationPower);
            dtMotors[3].setPower(lfPower - rotationPower);

            isFinishedTurning = (Math.abs(wanted.getAngle()) - 0.25 < Math.abs(-Gyro.getAngle()) && Math.abs(-Gyro.getAngle()) < Math.abs(wanted.getAngle()) + 0.25) && (Math.signum(wanted.getAngle()) == Math.signum(-Gyro.getAngle()) || wanted.getAngle() == 0);

        } else {

            if (wanted.getX() == 0) {

                moveRobotPID = new PID(DrivetrainConstants.moveRobotKp, DrivetrainConstants.moveRobotKi, DrivetrainConstants.moveRobotKd, DrivetrainConstants.moveRobotKf, DrivetrainConstants.moveRobotIzone, DrivetrainConstants.moveRobotMaxSpeed, DrivetrainConstants.moveRobotMinSpeed);

                lfCurrent = PoseTracker.getPose().getY();
                rfCurrent = PoseTracker.getPose().getY();


                final double lfError = lfWanted - lfCurrent;
                final double rfError = rfWanted - rfCurrent;


                if (lfError > rfError) {
                    moveRobotPID.setWanted(lfWanted);
                    lfPower = moveRobotPID.update(lfCurrent);
                    rfPower = (rfError / lfError) * lfPower;
                } else {
                    moveRobotPID.setWanted(rfWanted);
                    rfPower = moveRobotPID.update(rfCurrent);
                    lfPower = (lfError / rfError) * rfPower;
                }
                isFinishedLf = Math.abs(lfWanted) < Math.abs(lfCurrent);
                isFinishedRf = Math.abs(rfWanted) < Math.abs(rfCurrent);

                dtMotors[0].setPower(lfPower + rotationPower);
                dtMotors[1].setPower(rfPower + rotationPower);
                dtMotors[2].setPower(rfPower - rotationPower);
                dtMotors[3].setPower(lfPower - rotationPower);

            } else {

                moveRobotPID = new PID(DrivetrainConstants.moveRobotSideKp, DrivetrainConstants.moveRobotSideKi, DrivetrainConstants.moveRobotSideKd, DrivetrainConstants.moveRobotSideKf, DrivetrainConstants.moveRobotSideIzone, DrivetrainConstants.moveRobotSideMaxSpeed, DrivetrainConstants.moveRobotSideMinSpeed);

                lfCurrent = +PoseTracker.getPose().getX();
                rfCurrent = -PoseTracker.getPose().getX();


                final double lfError = lfWanted - lfCurrent;
                final double rfError = rfWanted - rfCurrent;


                moveRobotPID.setWanted(lfWanted);
                lfPower = moveRobotPID.update(lfCurrent);
                rfPower = -lfPower;


//                lfCurrent =  + PoseTracker.getPose().getX();
//                rfCurrent =  - PoseTracker.getPose().getX();

                isFinishedLf = Math.abs(lfWanted) < Math.abs(lfCurrent) ;
                isFinishedRf = Math.abs(rfWanted)  < Math.abs(rfCurrent);

                dtMotors[0].setPower((lfPower + rotationPower) * 1.3);
                dtMotors[1].setPower(rfPower + rotationPower);
                dtMotors[2].setPower((rfPower - rotationPower) * 1.3);
                dtMotors[3].setPower(lfPower - rotationPower);
            }

            isFinishedTurning = true;

        }


        PoseTracker.update();
//        }else {

        telemetry.addData("angle", Gyro.getAngle());
        telemetry.addData("wanted", wanted.getAngle());
//        }




        isFinished = isFinishedLf && isFinishedRf && isFinishedTurning;
        if (isFinished) {
            breakMotors();
            resetEncoders();
            PoseTracker.resetPos();
        }

    }

     */





}