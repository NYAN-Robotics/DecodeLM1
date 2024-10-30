package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Drive Forward 1 Second")
public class DriveForward1Second extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBackMotor");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontMotor");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBackMotor");




        waitForStart();

        leftFront.setPower(0.25);
        rightBack.setPower(0.25);
        rightFront.setPower(0.25);
        leftBack.setPower(0.25);
        sleep(1000);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);

    }

}
