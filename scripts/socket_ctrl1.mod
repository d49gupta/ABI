MODULE socket_comms
    VAR socketdev client_socket;
    VAR socketdev server_socket;
    VAR string received_msg;
    VAR string send_msg;
    VAR string pose_msg;
    VAR robtarget target_pose;
    VAR robtarget current_pose;
    VAR bool good_command;
    VAR bool good_data;
    VAR string client_sim_ip := "127.0.0.1";
    VAR string client_real_ip := "10.60.70.51";
    VAR num yaw_angle;
    VAR intnum comma_index;
    VAR num command_id;
    VAR string id_str;
    VAR string data_str;
    VAR pos move_data;
    !TASK PERS tooldata toolBladeTest:=[TRUE,[[69.2101,26.486,370.055],[0.204128,0.252974,0.0546959,-0.94411]],[3.613,[11,9.9,94.7],[1,0,0,0],0.017,0.018,0.005]];
    TASK PERS tooldata toolBladeTest := [TRUE, [[1.19, 1.1, 334.77], [1, 0, 0, 0]], [0.653, [11.99, -33.41, -0.98], [1, 0, 0, 0], 0, 0, 0]];
    VAR speeddata vSlowYaw := [10, 10, 1000, 1000];
    VAR num index := 1;
    PERS robtarget calibration_pose{4} := 
    [
    [[279.488,-19.312,-538.991],[0.000143494,0.965922,0.258832,-0.00011993],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,14724.8]],
    [[301.416,-19.7475,-539.054],[0.000945091,0.96583,0.259157,0.00313846],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,14750.8]],
    [[324.189,-19.8434,-539.222],[0.00163557,0.965729,0.259473,0.00622472],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,14776.2]],
    [[348.793,-20.0385,-539.401],[0.00235769,0.965603,0.259838,0.00950471],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,14803.3]]
    ];
    
    PERS robtarget Point1 := [[279.488,-19.312,-538.991],[0.000143494,0.965922,0.258832,-0.00011993],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,14724.8]];
    PERS robtarget Point2 := [[301.416,-19.7475,-539.054],[0.000945091,0.96583,0.259157,0.00313846],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,14750.8]];
    PERS robtarget Point3 := [[324.189,-19.8434,-539.222],[0.00163557,0.965729,0.259473,0.00622472],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,14776.2]];
    PERS robtarget Point4 := [[348.793,-20.0385,-539.401],[0.00235769,0.965603,0.259838,0.00950471],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,14803.3]];
    
    PROC openSocket()
        ActUnit CNV1;
        target_pose := CRobT(\Tool:=toolBladeTest \WObj:=wobj0); ! Default is tool0      
        SocketCreate server_socket;
        SocketBind server_socket, client_real_ip, 4000;
        SocketListen server_socket;
        SocketAccept server_socket, client_socket;
    ENDPROC
        
    PROC Send()
        current_pose := CRobT(\Tool:=toolBladeTest \WObj:=wobj0);
        
        pose_msg := ValToStr(current_pose.trans.x) + "," + 
                    ValToStr(current_pose.trans.y) + "," + 
                    ValToStr(current_pose.trans.z) + "," +
                    ValToStr(current_pose.rot.q1) + "," + 
                    ValToStr(current_pose.rot.q2) + "," + 
                    ValToStr(current_pose.rot.q3) + "," +
                    ValToStr(current_pose.rot.q4);
        send_msg := pose_msg + "\0A";
        SocketSend client_socket \Str:=send_msg;
    ENDPROC
    
    PROC Receive()
        SocketReceive client_socket \Str:=received_msg;
        comma_index := StrFind(received_msg, 1, ",");
        id_str := StrPart(received_msg, 1, comma_index - 1);
        good_command := StrToVal(id_str, command_id);
        
        IF good_command THEN
            TEST command_id
            CASE 1:
                data_str := "[" + StrPart(received_msg, comma_index + 1, StrLen(received_msg) - comma_index) + "]";
                good_data := StrToVal(data_str, move_data);
                MOVE_REL;
            CASE 2:
                StopMove;
                closeSocket;
            CASE 8:
                MOVE_CONVEYOR;
            CASE 9:
                STOP_CONVEYOR;
            CASE 7:
                RECORD_POINT;
            ENDTEST
        ENDIF
    ENDPROC
        
    PROC MOVE_REL()
        MoveL Offs(CRobT(\Tool:=toolBladeTest \WObj:=wobj0), move_data.x, move_data.y, move_data.z), v5, fine, toolBladeTest;
        !WaitRob\InPos;
    ENDPROC
    
    PROC closeSocket()
        SocketClose client_socket;
        SocketClose server_socket;
    ENDPROC
    
    PROC MOVE_CONVEYOR()
        ErrWRite\I,"Turning On CNV ","Turning On CNV";
        Set do_CNV_Fwd;
    ENDPROC
    
    PROC STOP_CONVEYOR()
        ErrWRite\I,"Turning Off CNV ","Turning Off CNV";
        reset do_CNV_Fwd;
    ENDPROC
    
    PROC RECORD_POINT()
        IF index >= 1 AND index <= 4 THEN
            calibration_pose{index} := CRobT(\Tool:=tool0 \WObj:=wobj0);
        ENDIF
        index := index + 1;
    ENDPROC
    
    PROC Calibrate()
       openSocket;
        
        WHILE TRUE DO
            Send;
            Receive;
       ENDWHILE
        
        !closeSocket;
    ENDPROC
    
ENDMODULE