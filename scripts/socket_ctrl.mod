MODULE socket_comms
    VAR socketdev client_socket;
    VAR socketdev server_socket;
    VAR string received_msg;
    VAR string send_msg;
    VAR robtarget target_pos;
    VAR robtarget current_pos;
    VAR num x_dest;
    VAR num y_dest;
    VAR num z_dest;
    VAR bool ok;
    VAR string client_ip := "127.0.0.1";

    PROC openSocket()
        target_pos := CRobT(\Tool:=tool0 \WObj:=wobj0);        
        SocketCreate server_socket;
        SocketBind server_socket, client_ip, 5000;
        SocketListen server_socket;
        SocketAccept server_socket, client_socket;
    ENDPROC
        
    PROC Send()
        current_pos := CRobT(\Tool:=tool0 \WObj:=wobj0);
        
        send_msg := ValToStr(current_pos.trans.x) + "," + 
                    ValToStr(current_pos.trans.y) + "," + 
                    ValToStr(current_pos.trans.z);
        
        SocketSend client_socket \Str:=send_msg;
    ENDPROC
    
    PROC Receive()
        SocketReceive client_socket \Str:=received_msg;
        
        IF StrPart(received_msg, 1, 2) = "X:" THEN
            ok := StrToVal(StrPart(received_msg, 3, StrLen(received_msg)-2), x_dest);
        ENDIF
        IF StrPart(received_msg, 1, 2) = "Y:" THEN
            ok := StrToVal(StrPart(received_msg, 3, StrLen(received_msg)-2), y_dest);
        ENDIF
        IF StrPart(received_msg, 1, 2) = "Z:" THEN
            ok := StrToVal(StrPart(received_msg, 3, StrLen(received_msg)-2), z_dest);
        ENDIF
        
        target_pos.trans.x := x_dest;
        target_pos.trans.y := y_dest;
        target_pos.trans.z := z_dest;
    ENDPROC
        
    PROC MOVE_REL()
        MoveL RelTool(CRobT(), x_dest, y_dest, z_dest), v10, fine, tool0;
    ENDPROC
    
    PROC MOVE_WORLD()
        MOVEJ target_pos, v10, fine, tool0;
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
            MOVE_REL;
        ENDWHILE
        
        closeSocket;
    ENDPROC
    
ENDMODULE