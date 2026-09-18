MODULE socket_comms
    VAR socketdev client_socket;
    VAR socketdev server_socket;
    VAR robtarget target_pose;
    VAR robtarget current_pose;
    VAR robtarget current_pose_world;
    VAR robtarget current_pose_conveyor;
    VAR string client_sim_ip := "127.0.0.1";
    VAR string client_real_ip := "10.60.70.51";
    VAR pos move_data;
    VAR bool pending_move := FALSE;
    VAR rawbytes recv_buf;
    VAR num cmd_id;
    VAR num fx;
    VAR num fy;
    VAR num fz;
    CONST num RECV_TIMEOUT := 0.02; ! short poll: don't let Send() stall just because no command is waiting yet
    CONST num MSG_IN_LEN := 16;  ! bytes: cmd_id, f1, f2, f3 as float32 each
    TASK PERS tooldata toolBladeTest := [TRUE, [[0, 0, 296.30], [1, 0, 0, 0]], [1.927, [4.838, 0.915, -156.08], [1, 0, 0, 0], 0, 0, 0]];
    VAR speeddata speed_var := [5, 50, 5000, 1000];
    VAR zonedata move_zone;
    VAR num index := 1;

    PERS pose uframe_test := [[0, 0, 0],[1, 0, 0, 0]];
    PERS wobjdata test_wobj := [FALSE, FALSE, "CNV1", [[0, 0, 0],[1, 0, 0, 0]],[[0, 0, 0],[1, 0, 0, 0]]];

    ! 4 point calibration
    PERS robtarget Point1 := [[368.352,34.0767,-956.827],[0.000920573,0.965966,0.258666,-0.000688489],[0,0,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];
    PERS robtarget Point2 := [[368.331,34.0901,-956.693],[0.000773514,0.965941,0.258762,-0.000601926],[0,0,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];
    PERS robtarget Point3 := [[368.435,33.9281,-956.709],[0.000363296,0.96592,0.258839,-0.00113627],[0,0,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];
    PERS robtarget Point4 := [[368.441,33.8682,-956.773],[0.00028639,0.965872,0.259019,-0.000901493],[0,0,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];

    ! 3 point calibration
    PERS robtarget Point5 := [[607.132,306.098,83.1973],[0.00164695,0.96611,0.25809,0.00417494],[0,0,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];
    PERS robtarget Point6 := [[660.685,305.53,81.667],[0.00162666,0.966068,0.258244,0.00441283],[0,0,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];
    PERS robtarget Point7 := [[608.48,357.725,81.69],[0.0022396,0.966065,0.258246,0.00478736],[0,0,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];

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
        VAR rawbytes send_buf;
        current_pose := CRobT(\Tool:=toolBladeTest \WObj:=wobj0);

        ClearRawBytes send_buf;
        PackRawBytes current_pose.trans.x, send_buf, RawBytesLen(send_buf) + 1 \Float4;
        PackRawBytes current_pose.trans.y, send_buf, RawBytesLen(send_buf) + 1 \Float4;
        PackRawBytes current_pose.trans.z, send_buf, RawBytesLen(send_buf) + 1 \Float4;
        PackRawBytes current_pose.extax.eax_f, send_buf, RawBytesLen(send_buf) + 1 \Float4;
        PackRawBytes speed_var.v_tcp, send_buf, RawBytesLen(send_buf) + 1 \Float4;

        SocketSend client_socket \RawData:=send_buf;
    ENDPROC

    FUNC bool TryReceiveOne()
        SocketReceive client_socket \RawData:=recv_buf \ReadNoOfBytes:=MSG_IN_LEN \Time:=RECV_TIMEOUT;
        RETURN TRUE;
        ERROR
            IF ERRNO = ERR_SOCK_TIMEOUT THEN
                RETURN FALSE;
            ENDIF
            RAISE;
    ENDFUNC

    PROC Receive()
        VAR bool got_one := FALSE;

        ! Drain the socket, dispatching every queued message as it's read (not just
        ! the last one) so a SetSpeed command isn't silently discarded by a MoveRel
        ! that arrived right behind it in the same drain pass. Only the final move
        ! target actually gets executed, once, after the drain completes.
        WHILE TryReceiveOne() DO
            got_one := TRUE;
            UnpackRawBytes recv_buf, 1, cmd_id \Float4;
            UnpackRawBytes recv_buf, 5, fx \Float4;
            UnpackRawBytes recv_buf, 9, fy \Float4;
            UnpackRawBytes recv_buf, 13, fz \Float4;

            DispatchMessage cmd_id, fx, fy, fz;
        ENDWHILE

        IF got_one AND pending_move THEN
            pending_move := FALSE;
            MOVE_REL;
        ENDIF
    ENDPROC

    PROC DispatchMessage(num cmd_id, num fx, num fy, num fz)
        TEST Round(cmd_id)
            CASE 1:
                move_data.x := fx;
                move_data.y := fy;
                move_data.z := fz;
                pending_move := TRUE;
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
                speed_var.v_tcp := fx;
                IF speed_var.v_tcp > 50 THEN
                    speed_var.v_tcp := 50;
                ELSEIF speed_var.v_tcp < 1 THEN
                    speed_var.v_tcp := 1;
                ENDIF
        ENDTEST
    ENDPROC

    PROC MOVE_REL()
        IF speed_var.v_tcp <= 5 THEN
            move_zone := fine;
        ELSEIF speed_var.v_tcp <= 15 THEN
            move_zone := z1;
        ELSE
            move_zone := z5;
        ENDIF
        ConfL \Off;
        MoveL Offs(CRobT(\Tool:=toolBladeTest \WObj:=wobj0), move_data.x, move_data.y, move_data.z), speed_var, move_zone, toolBladeTest;
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
        index := 1;
        test_wobj.oframe := [[0,0,0],[1,0,0,0]];
        openSocket;
        WHILE TRUE DO
            Send;
            Receive;
        ENDWHILE

        closeSocket;
    ENDPROC

ENDMODULE
