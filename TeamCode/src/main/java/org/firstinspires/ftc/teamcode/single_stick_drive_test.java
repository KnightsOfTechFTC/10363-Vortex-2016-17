package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Lego5 on 9/11/2016.
 */
@TeleOp (name = "Single Stick Driving test")
public class single_stick_drive_test extends OpMode {

    @Override
    public void init() {
        // init motors
        try {
            v_motor_left_drive=hardwareMap.dcMotor.get("left_drive");
        }
        catch (Exception p_exception)
        {
           v_motor_left_drive=null;
        }
        try {
            v_motor_right_drive=hardwareMap.dcMotor.get("left_drive");
        }
        catch (Exception p_exception)
        {
            v_motor_right_drive=null;
        }

    }


    @Override
    public void loop() {
        x=gamepad1.left_stick_x;
        y=gamepad1.left_stick_y;
        //find out what quadrant you're in
        if (x>=0&&y>=0){quadrant=1;}
        else if (x<=0&&y>=0){quadrant=2;}
        else if (x<=0&&y<=0){quadrant=3;}
        else {quadrant=4;}
        //find theta
        if (x==0&&y>=0){theta=90;}
        else if (x==0){theta=270;}
        else if (quadrant==1) {
            theta=Math.toDegrees(Math.atan(y/x));
        }
        else if (quadrant==2){

            theta=Math.toDegrees(Math.atan(y/x))+180;
        }
        else if (quadrant==3){
            theta=Math.toDegrees(Math.atan(y/x))+180;
        }
        else if (quadrant==4){
            theta=Math.toDegrees(Math.atan(y/x));
        }
        //find r
        r=Math.sqrt((x*x)+(y*y));

    }
    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_right_drive;
    private double x;
    private double y;
    private int quadrant;
    private double theta;
    private double r;

}
