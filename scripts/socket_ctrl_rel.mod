MODULE socket_comms
    VAR socketdev client_socket;
    VAR socketdev server_socket;
    VAR string received_msg;
    VAR string send_msg;
    VAR string pose_msg;
    VAR robtarget target_pose;
    VAR robtarget current_pose;
    VAR robtarget current_pose_world;
    VAR robtarget current_pose_conveyor;
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
    VAR string recv_buffer := "";
    VAR bool pending_move := FALSE;
    CONST num RECV_TIMEOUT := 5; ! seconds with no data at all before SocketReceive errors
    ! TASK PERS tooldata toolBladeTest:=[TRUE,[[69.2101,26.486,370.055],[0.204128,0.252974,0.0546959,-0.94411]],[3.613,[11,9.9,94.7],[1,0,0,0],0.017,0.018,0.005]];
    ! TASK PERS tooldata toolBladeTest := [TRUE, [[1.19, 1.1, 334.77], [1, 0, 0, 0]], [0.653, [11.99, -33.41, -0.98], [1, 0, 0, 0], 0, 0, 0]];
    TASK PERS tooldata toolBladeTest := [TRUE, [[0, 0, 296.30], [1, 0, 0, 0]], [1.927, [4.838, 0.915, -156.08], [1, 0, 0, 0], 0, 0, 0]];
    VAR speeddata speed_var := [5, 50, 5000, 1000];
    VAR num index := 1;
    
    PERS pose uframe_test := [[0, 0, 0],[1, 0, 0, 0]];
    PERS wobjdata test_wobj := [FALSE, FALSE, "CNV1", [[0, 0, 0],[1, 0, 0, 0]],[[0, 0, 0],[1, 0, 0, 0]]];
    
    ! 4 point calibration
    PERS robtarget Point1 := [[393.502,-3.39382,-861.45],[0.000212606,-0.965936,-0.258777,0.00110227],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1372.54]];
    PERS robtarget Point2 := [[502.345,-2.87438,-862.37],[0.000228094,0.96596,0.258692,0.000782558],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1481.3]];
    PERS robtarget Point3 := [[557.874,-2.59812,-862.78],[0.000502056,0.965943,0.258745,0.00244777],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1536.95]];
    PERS robtarget Point4 := [[616.88,-2.4332,-863.34],[0.000588622,0.965938,0.258749,0.00347404],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1596.06]];
    
    ! 3 point calibration
    PERS robtarget Point5 := [[616.88,-2.4332,-863.34],[0.000588622,0.965938,0.258749,0.00347404],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1596.06]];
    PERS robtarget Point6 := [[498.283,1.76193,-862.465],[0.000431789,0.965854,0.25908,0.00197032],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1475.83]];
    PERS robtarget Point7 := [[556.059,2.01758,-863.069],[0.000678125,0.96582,0.25919,0.00335087],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1533.1]];
    
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
        VAR intnum nl_pos;

        ! Append whatever was cut off last time until end character
        SocketReceive client_socket \Str:=received_msg \Time:=RECV_TIMEOUT;
        recv_buffer := recv_buffer + received_msg;

        ! iterate through all instances of end character for concated messages
        nl_pos := StrFind(recv_buffer, 1, "\0A");
        WHILE nl_pos <= StrLen(recv_buffer) DO
            DispatchMessage(StrPart(recv_buffer, 1, nl_pos - 1));
            recv_buffer := StrPart(recv_buffer, nl_pos + 1, StrLen(recv_buffer) - nl_pos);
            nl_pos := StrFind(recv_buffer, 1, "\0A");
        ENDWHILE

        IF pending_move THEN
            pending_move := FALSE;
            MOVE_REL;
        ENDIF
    ENDPROC

    PROC DispatchMessage(string msg)
        comma_index := StrFind(msg, 1, ",");
        id_str := StrPart(msg, 1, comma_index - 1);
        good_command := StrToVal(id_str, command_id);

        IF good_command THEN
            TEST command_id
            CASE 1:
                data_str := "[" + StrPart(msg, comma_index + 1, StrLen(msg) - comma_index) + "]";
                good_data := StrToVal(data_str, move_data);
                pending_move := good_data;
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
                WaitRob\InPos;
            CASE 6:
                data_str := StrPart(msg, comma_index + 1, StrLen(msg) - comma_index);
                good_data := StrToVal(data_str, speed_var.v_tcp);
                IF speed_var.v_tcp > 50 THEN
                    speed_var.v_tcp := 50;
                ELSEIF speed_var.v_tcp < 1 THEN
                    speed_var.v_tcp := 1;
                ENDIF
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
        Set do_CNV_Fwd;
    ENDPROC
    
    PROC STOP_CONVEYOR()
        ErrWRite\I,"Turning Off CNV ","Turning Off CNV";
        reset do_CNV_Fwd;
    ENDPROC
    
    PROC RECORD_POINT()
        WaitRob\InPos;
        current_pose_world := CRobT(\Tool:=toolBladeTest \WObj:=wobj0);
        current_pose_conveyor := CRobT(\Tool:=toolBladeTest \WObj:=test_wobj);
        IF index = 1 THEN
            Point1 := current_pose_world;
        ELSEIF index = 2 THEN
            Point2 := current_pose_world;
        ELSEIF index = 3 THEN
            Point3 := current_pose_world;
        ELSEIF index = 4 THEN
            Point4 := current_pose_world;
            Point5 := current_pose_conveyor;
        ELSEIF index = 5 THEN
            Point6 := current_pose_conveyor;
        ELSEIF index = 6 THEN
            Point7 := current_pose_conveyor;
        ENDIF
        index := index + 1;
    ENDPROC
    
    PROC Calibrate()
        index := 1; ! reset index for saved points
        test_wobj.oframe := [[0,0,0],[1,0,0,0]]; ! reset work object definition
        openSocket;
        WHILE TRUE DO
            Send;
            Receive;
       ENDWHILE
        
        closeSocket;
    ENDPROC
    
ENDMODULE