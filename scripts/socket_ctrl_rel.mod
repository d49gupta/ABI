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
    VAR speeddata speed_var := [5, 10, 1000, 1000];
    VAR num index := 1;
    PERS robtarget calibration_pose{4} := 
    [
    [[394.668,-0.180135,-861.621],[0.000378122,-0.965928,-0.25881,0.00118518],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1588.52]],
    [[451.328,0.141549,-862.07],[1.84214E-05,0.96591,0.258877,0.000474107],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1645.4]],
    [[501.739,0.271487,-862.509],[0.000348954,0.965911,0.258868,0.00185627],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1695.66]],
    [[551.475,0.391746,-863.218],[0.000789696,0.965924,0.258801,0.00335397],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1744.15]]
    ];
    
    PERS robtarget Point1 := [[394.668,-0.180135,-861.621],[0.000378122,-0.965928,-0.25881,0.00118518],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1588.52]];
    PERS robtarget Point2 := [[451.328,0.141549,-862.07],[1.84214E-05,0.96591,0.258877,0.000474107],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1645.4]];
    PERS robtarget Point3 := [[501.739,0.271487,-862.509],[0.000348954,0.965911,0.258868,0.00185627],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1695.66]];
    PERS robtarget Point4 := [[551.475,0.391746,-863.218],[0.000789696,0.965924,0.258801,0.00335397],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1744.15]];
    
    PROC openSocket()
        ActUnit CNV1;
        ClearWobj;
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
                    ValToStr(current_pose.rot.q4) + "," +
                    ValToStr(current_pose.extax.eax_f);
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
        MoveL Offs(CRobT(\Tool:=toolBladeTest \WObj:=wobj0), move_data.x, move_data.y, move_data.z), speed_var, fine, toolBladeTest;
        !WaitRob\InPos;
    ENDPROC
    
    PROC closeSocket()
        SocketClose client_socket;
        SocketClose server_socket;
    ENDPROC
    
    PROC MOVE_CONVEYOR()
        ErrWRite\I,"Turning On CNV ","Turning On CNV";
        speed_var := [10, 50, 5000, 1000];
        Set do_CNV_Fwd;
    ENDPROC
    
    PROC STOP_CONVEYOR()
        ErrWRite\I,"Turning Off CNV ","Turning Off CNV";
        speed_var := [5, 50, 5000, 1000];
        reset do_CNV_Fwd;
    ENDPROC
    
    PROC RECORD_POINT()
        WaitRob\InPos;
        current_pose := CRobT(\Tool:=toolBladeTest \WObj:=wobj0);
        IF index >= 1 AND index <= 4 THEN
            calibration_pose{index} := current_pose;
        ENDIF
        IF index = 1 THEN
            Point1 := current_pose;
        ELSEIF index = 2 THEN
            Point2 := current_pose;
        ELSEIF index = 3 THEN
            Point3 := current_pose;
        ELSEIF index = 4 THEN
            Point4 := current_pose;
        ENDIF
        index := index + 1;
    ENDPROC
    
    PROC Calibrate()
        openSocket;
        WHILE TRUE DO
            Send;
            Receive;
       ENDWHILE
        
        closeSocket;
    ENDPROC
    
ENDMODULE