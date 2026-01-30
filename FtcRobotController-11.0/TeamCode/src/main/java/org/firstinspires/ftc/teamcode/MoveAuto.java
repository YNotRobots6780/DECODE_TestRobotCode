/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Move Auto", group="Robot")
//@Disabled
public class MoveAuto extends LinearOpMode {

    private IMU imu = null;
    private boolean isFlywheelUpToSpeed = false;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontIntake = null;
    private DcMotor middleIntake = null;
    private DcMotorEx outake = null;

    private DcMotorEx outake2 = null;

    private ElapsedTime     runtime = new ElapsedTime();

    double y = 0; // Remember, Y stick is reversed!
    double x = 0; // Counteract imperfect strafing
    double rx = 0;

    double turnSpeedFactor = 0.8; // Adjust this value between 0 (no turn) and 1 (full turn)
    double scaledRx = rx * turnSpeedFactor; // smoother turning

    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(scaledRx), 1);

    boolean isOuttakeOn = false;
    boolean isOuttakePressed = false;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        imu = hardwareMap.get(IMU.class, "imu");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontIntake = hardwareMap.get(DcMotor.class, "frontintake");
        middleIntake = hardwareMap.get(DcMotor.class, "middle_intake");

        outake = (DcMotorEx) hardwareMap.get(DcMotor.class, "outake");
        outake2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "outake2");

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        outake.setDirection(DcMotorSimple.Direction.REVERSE);
        middleIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        //outake2.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);



        outake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RevHubOrientationOnRobot revHubOrintation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(revHubOrintation));

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        imu.resetYaw();

        // Wait for the game to start (driver presses START)
        waitForStart();




        // double x = (xInput * Math.sin(-heading)) - (yInput * Math.cos(-heading));
        // double y = (xInput * Math.cos(-heading)) + (yInput * Math.sin(-heading));

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        frontLeftDrive.setPower((y + x + scaledRx) / denominator * .9);
        backLeftDrive.setPower((y - x + scaledRx) / denominator * .9);
        frontRightDrive.setPower((y - x - scaledRx) / denominator *.9);
        backRightDrive.setPower((y + x - scaledRx) / denominator * .9);

        // Step 1:  Drive forward for 3 seconds

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            y= -0.5;

        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 30)) {
            y= 0;

        }


        }




    }

