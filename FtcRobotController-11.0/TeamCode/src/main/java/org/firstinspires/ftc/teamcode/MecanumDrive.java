
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="Mecanum Drive Example", group="Iterative Opmode")
public class MecanumDrive extends OpMode
{

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor frontLeftMotor  = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor   = null;
    private DcMotor backRightMotor  = null;
    private DcMotor frontMotorIntake = null;
    private DcMotor middleMotorIntake = null;
    private DcMotor outakeMotor = null;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor  = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor    = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor   = hardwareMap.get(DcMotor.class, "back_right");
        frontMotorIntake  = hardwareMap.get(DcMotor.class, "front_intake");
        middleMotorIntake = hardwareMap.get(DcMotor.class, "middle_intake");
        outakeMotor = hardwareMap.get(DcMotor.class, "outake");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // frontMotorIntake.setDirection(DcMotor.Direction.);
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        outakeMotor.setPower(.7);

        if (gamepad1.a)    {
            frontMotorIntake.setPower(1.0);
            middleMotorIntake.setPower(1.0);
        }
        else {
            frontMotorIntake.setPower(0);
            middleMotorIntake.setPower(0);
        }



        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);

    }

}