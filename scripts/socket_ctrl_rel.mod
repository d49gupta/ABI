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
    VAR speeddata speed_var := [50, 10, 1000, 1000];
    VAR num index := 1;
    PERS robtarget calibration_pose{4} := 
    [
    [[477.54,-11.2529,-862.514],[0.000344298,0.96591,0.258874,0.00109268],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]],
    [[426,-11.9398,-862.232],[5.10992E-05,0.965888,0.258959,0.000114794],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]],
    [[478.843,-62.8853,-862.553],[0.000173246,0.965917,0.25885,0.00132356],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]],
    [[490.73,-0.40911,-756.962],[1.6634E-05,-0.965946,-0.258745,-0.000101222],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,284.637]]
    ];
    
    PERS pose uframe_test := [[100, 0, 0],[1, 0, 0, 0]];
    PERS wobjdata test_wobj := [FALSE, FALSE, "", [[0, 0, 0],[1, 0, 0, 0]],[[0, 0, 0],[1, 0, 0, 0]]];
    
    PERS robtarget Point1 := [[477.54,-11.2529,-862.514],[0.000344298,0.96591,0.258874,0.00109268],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];
    PERS robtarget Point2 := [[426,-11.9398,-862.232],[5.10992E-05,0.965888,0.258959,0.000114794],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];
    PERS robtarget Point3 := [[478.843,-62.8853,-862.553],[0.000173246,0.965917,0.25885,0.00132356],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];
    PERS robtarget Point4 := [[490.73,-0.40911,-756.962],[1.6634E-05,-0.965946,-0.258745,-0.000101222],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,284.637]];
    
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
            CASE 5:
                GoHomeJ;
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