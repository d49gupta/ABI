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
    [[428.351,0.414514,-861.818],[0.00046544,-0.965901,-0.258906,0.00135864],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1605.99]],
    [[478.073,0.698729,-862.052],[1.67996E-05,-0.965876,-0.259003,-0.000787871],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1656.25]],
    [[534.077,-1.23022,-862.791],[0.000749781,0.965906,0.258878,0.0028275],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,107.143]],
    [[582.534,-1.02256,-863.246],[0.001097,0.965878,0.258958,0.00443628],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,155.249]]
    ];
    
    PERS robtarget Point1 := [[427.787,-1.53588,-861.851],[0.000354235,-0.965932,-0.258792,0.00141893],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];
    PERS robtarget Point2 := [[476.875,-1.41708,-862.437],[0.000190237,0.965926,0.258816,0.000693373],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,49.6459]];
    PERS robtarget Point3 := [[534.077,-1.23022,-862.791],[0.000749781,0.965906,0.258878,0.0028275],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,107.143]];
    PERS robtarget Point4 := [[582.534,-1.02256,-863.246],[0.001097,0.965878,0.258958,0.00443628],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,155.249]];
    
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
        
        !closeSocket;
    ENDPROC
    
ENDMODULE