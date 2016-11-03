package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Lego5 on 10/16/2016.
 */
@TeleOp(name = "10363 Competition TeleOp (with Single-Stick Driving")
public class Team_10363_TeleOp_with_single_stick_driving extends OpMode {
    /* RIP Robert the Robot 2015-2016. May his Res-Q skills be remembered by his 2 children,
        Robert II (a temp name) and JVBot (also a temp name) and his lifelong friend 9924Bot.  */
    Team_10363_TeleOp_Decs RobertII= new Team_10363_TeleOp_Decs();
    @Override
    public void init() {

        RobertII.init(hardwareMap);
    }

    @Override
    public void loop() {
        //update telemetry
        telemetry.update();
        //set motor powers using single-stick driving methods
        leftPower=RobertII.single_stick_drive_left(gamepad1.left_stick_x,gamepad1.left_stick_y);
        rightPower=RobertII.single_stick_drive_right(gamepad1.left_stick_x,gamepad1.left_stick_y);
        RobertII.setDrivePowerWithCorrection((float) rightPower, (float) leftPower);
        //add motor powers to telemetry
        telemetry.addData("1: Left Drive Motor Power: ", leftPower);
        telemetry.addData("2: Right Drive Motor Power: ", rightPower);
        if (gamepad1.x&&!x_press){
            x_press=true;
            mode++;
            if(mode==2){mode=-1;}
        }
        if (!gamepad1.x){
            x_press=false;
        }
        RobertII.m_intake_power(mode);
        telemetry.addData("3: intake speed: ",mode);
        if (gamepad1.y&&!y_press){
            y_press=true;
            liftmode++;
            if(liftmode==2){liftmode=-1;}
        }
        if (!gamepad1.y){
            y_press=false;
        }
        RobertII.m_lift_power(liftmode);
        telemetry.addData("4: lift speed: ",liftmode);
        if (gamepad1.a&&!a_press){
            a_press=true;
            beacons=!beacons;
        }
        if (!gamepad1.a){
            a_press=false;
        }
        RobertII.press_or_reset_beacons(beacons);
    }
    double leftPower;
    double rightPower;
    int liftmode=1;
    boolean y_press;
    boolean beacons;
    boolean a_press;
    int mode=1;
    boolean x_press;

}
