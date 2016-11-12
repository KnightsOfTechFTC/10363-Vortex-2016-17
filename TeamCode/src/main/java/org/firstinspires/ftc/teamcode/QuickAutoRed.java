package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Lego5 on 11/5/2016.
 */
@Disabled
@Autonomous (name="Quick Autonomous(red)")
public class QuickAutoRed extends OpMode {
    Team_10363_Auto_Decs RobTest= new Team_10363_Auto_Decs();
    @Override
    public void init() {
        RobTest.init(hardwareMap);
        state=0;
        encoderLeft=RobTest.a_left_encoder_pos();
        encoderRight=RobTest.a_right_encoder_pos();
    }
    @Override
    public void loop() {
        telemetry.addData("state ",state);
        telemetry.addData("left-encoder",RobTest.a_left_encoder_pos());
        telemetry.addData("right-encoder",RobTest.a_right_encoder_pos());
        telemetry.addData("ground white", RobTest.a_ground_alpha());
        telemetry.addData("heading",RobTest.a_gyro_heading());
        switch (state){
            case 0:
                RobTest.setDrivePower(-.3f,.3f);
                if (RobTest.have_drive_encoders_reached(encoderLeft+1440,-2440+encoderRight,true)) {
                    RobTest.setDrivePower(0, 0);
                    encoderLeft=RobTest.a_left_encoder_pos();
                    encoderRight=RobTest.a_right_encoder_pos();
                    state++;
                }
                break;
            case 1:
                RobTest.setDrivePower(((float) (-.3 + RobTest.adjspeed(1, RobTest.a_gyro_heading()-315))), (float) (-.3 - RobTest.adjspeed(1, RobTest.a_gyro_heading()-315)));
                if (RobTest.a_ground_alpha()>=7&&RobTest.have_drive_encoders_reached(20000+encoderLeft,20000+encoderRight,true)){
                    RobTest.setDrivePower(0,0);
                    encoderLeft=RobTest.a_left_encoder_pos();
                    encoderRight=RobTest.a_right_encoder_pos();
                    state++;
                }
                break;
            case 2:
                if (RobTest.a_ground_alpha()>=7) {
                    RobTest.setDrivePower(-.3f, -.3f);
                }
                else if (RobTest.a_gyro_heading()<270){
                    RobTest.setDrivePower(-.3f,.3f);
                }
                else if (RobTest.a_gyro_heading()>270){RobTest.setDrivePower(-.3f,.3f);}
                else {RobTest.setDrivePower(-.3f,-.3f);}
                if (RobTest.have_drive_encoders_reached(2000+encoderLeft,2000+encoderRight,true)){
                    RobTest.setDrivePower(0,0);
                    encoderLeft=RobTest.a_left_encoder_pos();
                    encoderRight=RobTest.a_right_encoder_pos();
                    RobTest.move_left_beacon_to_read();
                    red=RobTest.a_left_red();
                    blue=RobTest.a_left_blue();
                    state++;
                }
                break;
            case 3:
                if (red>2&&blue<2){
                    RobTest.reset_left_beacon_servo();
                    state++;
                }
                else if (red<2&&blue>2){
                    RobTest.reset_left_beacon_servo();
                    RobTest.push_right_beacon();
                    state=state+2;
                }
                else {state=state+2;}
                break;
            case 4:
                RobTest.push_left_beacon();
                RobTest.setDrivePower(.2f,.2f);
                if (RobTest.have_drive_encoders_reached(encoderLeft-20,encoderRight-20,false)){
                    RobTest.setDrivePower(0,0);
                    encoderLeft=RobTest.a_left_encoder_pos();
                    encoderRight=RobTest.a_right_encoder_pos();
                    state++;
                }
                break;
            case 5:
                RobTest.setDrivePower(.3f,.3f);
                if (RobTest.have_drive_encoders_reached(encoderLeft-700,encoderRight-700,false)){
                    RobTest.setDrivePower(0,0);
                    encoderLeft=RobTest.a_left_encoder_pos();
                    encoderRight=RobTest.a_right_encoder_pos();
                    state++;
                }
            }
        }


    int state;
    int encoderLeft;
    int encoderRight;
    double red;
    double blue;
}