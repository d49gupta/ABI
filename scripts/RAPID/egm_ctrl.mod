MODULE EGM_Cartesian_Module
    VAR egmident egmID1;
    
    LOCAL CONST pose base_frame := [[0, 0, 0], [1, 0, 0, 0]];
    CONST egm_minmax egm_minmax_lin1:=[-100,100]; !in mm
    CONST egm_minmax egm_minmax_rot1:=[-5,5];! in degees
    

    PROC main()
        EGMGetId egmID1;
        EGMSetupUC ROB_1, egmID1, "conf1", "UCdevice:" \Pose \CommTimeout:=1;
        EGMStreamStart egmID1;
        
        EGMActPose egmID1 \Tool:=tool0 \WObj:=wobj0, base_frame, EGM_FRAME_WOBJ, base_frame, EGM_FRAME_WOBJ;
        
        EGMRunPose egmID1, EGM_STOP_HOLD \x \y \z \rx \ry \rz \CondTime:=2000000 \RampInTime:=0.01 \PosCorrGain:=1;
        
        EGMStop egmID1, EGM_STOP_HOLD;
    ENDPROC
ENDMODULE