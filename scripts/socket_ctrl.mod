MODULE SocketSync
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

    PROC main()
        ! Start at current location to avoid jumps
        target_pos := CRobT(\Tool:=tool0 \WObj:=wobj0);
        
        SocketCreate server_socket;
        SocketBind server_socket, client_ip, 5000;
        SocketListen server_socket;
        SocketAccept server_socket, client_socket;

        WHILE TRUE DO
            ! 1. RECEIVE: Expecting "X,Y,Z"
            
            ! 2. SEND BACK: Get actual position after move
            current_pos := CRobT(\Tool:=tool0 \WObj:=wobj0);
            
            send_msg := ValToStr(current_pos.trans.x) + "," + 
                        ValToStr(current_pos.trans.y) + "," + 
                        ValToStr(current_pos.trans.z);
            
            SocketSend client_socket \Str:=send_msg;
            SocketReceive client_socket \Str:=received_msg;
            
            ! Parsing logic (Simple CSV)
            ! Note: In production, use a more robust split function
            ok := StrToVal(StrPart(received_msg, 1, StrFind(received_msg,1,",")-1), x_dest);
            ! For a Capstone, you may want to send 3 separate socket calls 
            ! or use a fixed-length string for easier parsing.
            
            ! Assign and Move
            target_pos.trans.x := x_dest;
            target_pos.trans.y := y_dest;
            target_pos.trans.z := z_dest;
            
            MoveL RelTool(CRobT(), x_dest, y_dest, z_dest), v10, fine, tool0;
        ENDWHILE
    ERROR
        SocketClose client_socket;
        SocketClose server_socket;
        RETRY;
    ENDPROC
ENDMODULE