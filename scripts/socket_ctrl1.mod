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
    [[387.916,-1.7782,-526.849],[0.000582474,-0.965917,-0.25884,0.00232643],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,224.908]],
    [[441.821,-1.57647,-527.306],[0.000123586,0.965906,0.258894,0.000426359],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,280.326]],
    [[490.344,-1.60661,-527.569],[0.000535485,0.965884,0.258963,0.00257265],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,330.434]],
    [[545.898,-1.44915,-528.367],[0.00119476,0.965885,0.258919,0.00508069],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,387.777]]
    ];
    
    PERS robtarget Point1 := [[430.543,-2.05962,-526.977],[0.000377543,-0.965944,-0.258744,0.001748],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,308.113]];
    PERS robtarget Point2 := [[478.03,-1.93233,-527.597],[0.000102451,0.965926,0.258817,0.000382491],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,356.373]];
    PERS robtarget Point3 := [[529.291,-1.82707,-527.949],[0.000594213,0.965909,0.258868,0.00253709],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,410.176]];
    PERS robtarget Point4 := [[576.803,-1.73733,-528.626],[0.00081679,0.965904,0.258867,0.00397386],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,458.359]];
    
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
        IF index >= 1 AND index <= 4 THEN
            calibration_pose{index} := CRobT(\Tool:=toolBladeTest \WObj:=wobj0);
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