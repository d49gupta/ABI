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
    [[391.583,1.15448,-861.384],[0.000295523,-0.965905,-0.258893,0.0012451],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1369.15]],
    [[439.646,1.45629,-861.949],[0.000119628,0.965879,0.258992,0.0003783],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1416.87]],
    [[498.283,1.76193,-862.465],[0.000431789,0.965854,0.25908,0.00197032],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1475.83]],
    [[556.059,2.01758,-863.069],[0.000678125,0.96582,0.25919,0.00335087],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1533.1]]
    ];
    
    PERS pose uframe_test := [[0, 0, 0],[1, 0, 0, 0]];
    PERS wobjdata test_wobj := [FALSE, FALSE, "CNV1", [[0, 0, 0],[1, 0, 0, 0]],[[492.843, -5.70361, -862.435],[0.00386325, -0.00409188, 0.000501966, -0.999984]]];
    
    PERS robtarget Point1 := [[393.502,-3.39382,-861.45],[0.000212606,-0.965936,-0.258777,0.00110227],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1372.54]];
    PERS robtarget Point2 := [[502.345,-2.87438,-862.37],[0.000228094,0.96596,0.258692,0.000782558],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1481.3]];
    PERS robtarget Point3 := [[557.874,-2.59812,-862.78],[0.000502056,0.965943,0.258745,0.00244777],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1536.95]];
    PERS robtarget Point4 := [[616.88,-2.4332,-863.34],[0.000588622,0.965938,0.258749,0.00347404],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1596.06]];
    PERS robtarget Point5 := [[498.283,1.76193,-862.465],[0.000431789,0.965854,0.25908,0.00197032],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1475.83]];
    PERS robtarget Point6 := [[556.059,2.01758,-863.069],[0.000678125,0.96582,0.25919,0.00335087],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1533.1]];
    
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
        IF index = 1 THEN
            Point1 := current_pose;
        ELSEIF index = 2 THEN
            Point2 := current_pose;
        ELSEIF index = 3 THEN
            Point3 := current_pose;
        ELSEIF index = 4 THEN
            Point4 := current_pose;
        ELSEIF index = 5 THEN
            Point5 := current_pose;
        ELSEIF index = 6 THEN
            Point6 := current_pose;
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