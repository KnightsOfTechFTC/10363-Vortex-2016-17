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
        RobertII.setDrivePowerWithCorrection((float) leftPower, (float) rightPower);
        //add motor powers to telemetry
        telemetry.addData("1: Left Drive Motor Power: ", leftPower);
        telemetry.addData("2: Right Drive Motor Power: ", rightPower);
        //set intake servo's mode
        if (gamepad2.a&&!a_press){
            a_press=true;
            if (intake_mode<=1){
                intake_mode=-1;
            }
            else {
                intake_mode++;
            }
        }
        if (!gamepad2.a){
            a_press=false;
        }
        RobertII.m_sweep_speed(intake_mode);
        telemetry.addData("3:Intake servo speed: ", intake_mode);

    }
    double leftPower;
    double rightPower;
    int intake_mode=1;
    boolean a_press=false;

}
