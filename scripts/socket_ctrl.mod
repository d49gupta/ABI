MODULE socket_comms
    VAR socketdev client_socket;
    VAR socketdev server_socket;
    VAR string received_msg;
    VAR string send_msg;
    VAR robtarget target_pose;
    VAR robtarget current_pose;
    VAR num x_dest;
    VAR num y_dest;
    VAR num z_dest;
    VAR bool good_command;
    VAR bool good_data;
    VAR string client_ip := "127.0.0.1";
    
    VAR intnum comma_index;
    VAR num command_id;
    VAR string id_str;
    VAR string data_str;
    VAR pos move_data;
    TASK PERS tooldata toolBladeTest:=[TRUE,[[69.2101,26.486,370.055],[0.204128,0.252974,0.0546959,-0.94411]],[3.613,[11,9.9,94.7],[1,0,0,0],0.017,0.018,0.005]];

    
    PROC openSocket()
        target_pose := CRobT(\Tool:=toolBladeTest \WObj:=wobj0); ! Default is tool0      
        SocketCreate server_socket;
        SocketBind server_socket, client_ip, 5000;
        SocketListen server_socket;
        SocketAccept server_socket, client_socket;
    ENDPROC
        
    PROC Send()
        current_pose := CRobT(\Tool:=toolBladeTest \WObj:=wobj0);
        
        send_msg := ValToStr(current_pose.trans.x) + "," + 
                    ValToStr(current_pose.trans.y) + "," + 
                    ValToStr(current_pose.trans.z) + "," +
                    ValToStr(current_pose.rot.q1) + "," + 
                    ValToStr(current_pose.rot.q2) + "," + 
                    ValToStr(current_pose.rot.q3) + "," +
                    ValToStr(current_pose.rot.q4);
        
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
            CASE 3:
                data_str := "[" + StrPart(received_msg, comma_index + 1, StrLen(received_msg) - comma_index) + "]";
                good_data := StrToVal(data_str, move_data);
                target_pose.trans.x := move_data.x;
                target_pose.trans.y := move_data.y;
                target_pose.trans.z := move_data.z;
                MOVE_BODY;
            ENDTEST
        ENDIF
    ENDPROC
        
    PROC MOVE_REL()
        MoveL Offs(CRobT(\Tool:=toolBladeTest \WObj:=wobj0), move_data.x, move_data.y, move_data.z), v10, fine, toolBladeTest;
    ENDPROC
    
    PROC MOVE_WORLD()
        MOVEJ target_pose, v40, fine, tool0;
    ENDPROC
    
    PROC MOVE_BODY()
        MOVEL target_pose, v50, z1, toolBladeTest;
    ENDPROC
    
    PROC closeSocket()
        SocketClose client_socket;
        SocketClose server_socket;
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