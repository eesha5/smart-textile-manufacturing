(* ======================================================================
   SMART TEXTILE MANUFACTURING - PLC PROGRAM v2 (IEC 61131-3 ST)
   Pipelined indexing line with emergency stop.
   Generated from the same rung list that the web simulator executes, so
   the simulator and this program behave identically.

   Task interval: 20 ms. The takt stopwatch rung adds 20 ms per scan - change
   that constant if you change the task interval.

   Address profile: SIMULATION - field inputs are Modbus coils 816 to 832 (%QX102.0 ...)
   ====================================================================== *)

PROGRAM SmartTextile
  VAR
    (* ---- located variables: I/O image, HMI bits, status, KPI registers ---- *)
    (* Field inputs *)
    X0_StartPB AT %QX102.0 : BOOL;  (* X0: Start push-button (physical) *)
    X1_StopPB AT %QX102.1 : BOOL;  (* X1: Stop push-button (physical) *)
    X2_ResetPB AT %QX102.2 : BOOL;  (* X2: Reset push-button (physical) *)
    X3_AtS1_FeedId AT %QX102.3 : BOOL;  (* X3: Product at station 1: feed / identification *)
    X4_TypeA_Sensor AT %QX102.4 : BOOL;  (* X4: Type A sensor *)
    X5_TypeB_Sensor AT %QX102.5 : BOOL;  (* X5: Type B sensor *)
    X6_AtS2_Cutting AT %QX102.6 : BOOL;  (* X6: Product at station 2: cutting *)
    X7_AtS3_Finishing AT %QX102.7 : BOOL;  (* X7: Product at station 3: finishing *)
    X10_AtS4_Inspection AT %QX103.0 : BOOL;  (* X10: Product at station 4: inspection *)
    X11_InspComplete AT %QX103.1 : BOOL;  (* X11: Inspection complete (vision result valid) *)
    X12_DefectDetected AT %QX103.2 : BOOL;  (* X12: Defect detected by vision *)
    X13_RobotReady AT %QX103.3 : BOOL;  (* X13: Robot ready (home position) *)
    X14_RobotPlaceDone AT %QX103.4 : BOOL;  (* X14: Robot place done *)
    X15_PackComplete AT %QX103.5 : BOOL;  (* X15: Packaging complete *)
    X16_RobotHealthy AT %QX103.6 : BOOL;  (* X16: Robot healthy (NC wiring: ON = OK, OFF = fault) *)
    X17_EStopOK AT %QX103.7 : BOOL;  (* X17: Emergency stop chain (NC wiring: ON = released, OFF = pressed) *)
    X20_AtS5_Robot AT %QX104.0 : BOOL;  (* X20: Product at station 5: robot pick *)
    (* Field outputs *)
    Y0_FeederMotor AT %QX0.0 : BOOL;  (* Y0: Feeder motor *)
    Y1_MainConveyor AT %QX0.1 : BOOL;  (* Y1: Main conveyor *)
    Y2_Cutter AT %QX0.2 : BOOL;  (* Y2: Cutter (stage 1) *)
    Y3_HeatPress AT %QX0.3 : BOOL;  (* Y3: Heat press (stage 2) *)
    Y4_InspTrigger AT %QX0.4 : BOOL;  (* Y4: Inspection trigger (camera) *)
    Y5_RobotPickReq AT %QX0.5 : BOOL;  (* Y5: Robot pick request *)
    Y6_DestGood AT %QX0.6 : BOOL;  (* Y6: Robot destination: good *)
    Y7_DestReject AT %QX0.7 : BOOL;  (* Y7: Robot destination: reject *)
    Y10_PackMachine AT %QX1.0 : BOOL;  (* Y10: Packaging machine *)
    Y11_AlarmBuzzer AT %QX1.1 : BOOL;  (* Y11: Alarm buzzer *)
    Y12_RunLamp AT %QX1.2 : BOOL;  (* Y12: Run lamp (green) *)
    Y13_FaultLamp AT %QX1.3 : BOOL;  (* Y13: Fault / E-stop lamp (red) *)
    (* HMI bits *)
    M100_HmiStart AT %QX100.0 : BOOL;  (* M100: HMI start button *)
    M101_HmiStop AT %QX100.1 : BOOL;  (* M101: HMI stop button *)
    M102_HmiReset AT %QX100.2 : BOOL;  (* M102: HMI reset button *)
    M103_HmiAuto AT %QX100.3 : BOOL;  (* M103: HMI auto mode button *)
    M104_HmiManual AT %QX100.4 : BOOL;  (* M104: HMI manual mode button *)
    M105_HmiProdA AT %QX100.5 : BOOL;  (* M105: HMI product A override *)
    M106_HmiProdB AT %QX100.6 : BOOL;  (* M106: HMI product B override *)
    M107_HmiClear AT %QX100.7 : BOOL;  (* M107: HMI clear counters button *)
    M108_HmiEStop AT %QX101.0 : BOOL;  (* M108: HMI emergency stop button *)
    (* Internal relays *)
    M0_SystemRun AT %QX105.0 : BOOL;  (* M0: System run *)
    M1_Fault AT %QX105.1 : BOOL;  (* M1: Fault latched *)
    M2_EStop AT %QX105.2 : BOOL;  (* M2: Emergency stop latched *)
    M31_ManualMode AT %QX105.3 : BOOL;  (* M31: Manual mode (0 = auto) *)
    M17_ManualTypeB AT %QX105.4 : BOOL;  (* M17: Manual product selection (0 = A, 1 = B) *)
    M3_PhaseIndex AT %QX105.5 : BOOL;  (* M3: Phase: conveyor indexing *)
    M4_PhaseWork AT %QX105.6 : BOOL;  (* M4: Phase: stations working *)
    M75_PackBusy AT %QX105.7 : BOOL;  (* M75: Packaging machine busy *)
    M50_S1_Occupied AT %QX106.0 : BOOL;  (* M50: Station 1 occupied *)
    M51_S2_Occupied AT %QX106.1 : BOOL;  (* M51: Station 2 occupied *)
    M52_S3_Occupied AT %QX106.2 : BOOL;  (* M52: Station 3 occupied *)
    M53_S4_Occupied AT %QX106.3 : BOOL;  (* M53: Station 4 occupied *)
    M54_S5_Occupied AT %QX106.4 : BOOL;  (* M54: Station 5 occupied *)
    M55_DefectAtS4 AT %QX106.5 : BOOL;  (* M55: Defect flag of the product at station 4 *)
    M56_DefectAtS5 AT %QX106.6 : BOOL;  (* M56: Defect flag of the product at station 5 *)
    M60_S1_IsTypeB AT %QX107.0 : BOOL;  (* M60: Product at station 1 is type B *)
    M61_S2_IsTypeB AT %QX107.1 : BOOL;  (* M61: Product at station 2 is type B *)
    M62_S3_IsTypeB AT %QX107.2 : BOOL;  (* M62: Product at station 3 is type B *)
    M63_S4_IsTypeB AT %QX107.3 : BOOL;  (* M63: Product at station 4 is type B *)
    M64_S5_IsTypeB AT %QX107.4 : BOOL;  (* M64: Product at station 5 is type B *)
    (* Data registers *)
    D100_GoodCount AT %MW0 : INT;  (* D100: Good count *)
    D101_RejectCount AT %MW1 : INT;  (* D101: Reject count *)
    D102_TotalCount AT %MW2 : INT;  (* D102: Total count *)
    D103_DefectPctX100 AT %MW3 : INT;  (* D103: Defect percentage x100 (1670 = 16.70 %) *)
    D104_TaktTime01s AT %MW4 : INT;  (* D104: Takt time: gap between sorted products (0.1 s) *)
    D105_RateX10PerMin AT %MW5 : INT;  (* D105: Production rate (0.1 products / min) *)
    D106_CurrentProduct AT %MW6 : INT;  (* D106: Product at station 1 (1 = A, 2 = B) *)
    D107_Phase AT %MW7 : INT;  (* D107: Phase (0 idle, 10 indexing, 20 working) *)
    D108_FaultCode AT %MW8 : INT;  (* D108: Fault code (0 none, 1-5) *)
    D109_PackagedCount AT %MW9 : INT;  (* D109: Packaged count *)
  END_VAR
  VAR
    (* ---- internal variables (not visible to Modbus) ---- *)
    M69_FeedDone : BOOL;  (* M69: Feed time elapsed in this work phase *)
    M70_S1_IdDone : BOOL;  (* M70: Station 1 identification done *)
    M71_S2_CutDone : BOOL;  (* M71: Station 2 cutting done *)
    M72_S3_FinishDone : BOOL;  (* M72: Station 3 finishing done *)
    M73_S4_InspDone : BOOL;  (* M73: Station 4 inspection result latched *)
    M80_S1_Departed : BOOL;  (* M80: Old product has left station 1 (this index) *)
    M81_S2_Departed : BOOL;  (* M81: Old product has left station 2 (this index) *)
    M82_S3_Departed : BOOL;  (* M82: Old product has left station 3 (this index) *)
    M83_S4_Departed : BOOL;  (* M83: Old product has left station 4 (this index) *)
    M84_S5_Departed : BOOL;  (* M84: Old product has left station 5 (this index) *)
    M22_GoodSorted : BOOL;  (* M22: Good product sorted (pulse) *)
    M23_RejectSorted : BOOL;  (* M23: Reject product sorted (pulse) *)
    M24_IndexComplete : BOOL;  (* M24: Index complete: all products at next station *)
    M25_WorkComplete : BOOL;  (* M25: Work complete: all stations finished *)
    M26_StationsDone : BOOL;  (* M26: All occupied stations finished their task *)
    M28_AllDeparted : BOOL;  (* M28: Every old product has left its station *)
    M29_AllArrived : BOOL;  (* M29: Every expected product reached its station *)
    M32_PackagedPulse : BOOL;  (* M32: Product packaged (pulse) *)
    D110_CycleMs : DINT;  (* D110: Takt stopwatch (ms, scan based) *)
    D111_RunMs : DINT;  (* D111: Running time stopwatch (ms, scan based) *)
  END_VAR
  VAR
    (* Timers and counters (IEC TON / CTU) *)
    T0_FeedTime : TON;  (* T0: Feed time 2 s *)
    T1_CutTime : TON;  (* T1: Cutting time 3 s *)
    T2_FinishA : TON;  (* T2: Finishing time, product A 5 s *)
    T3_FinishB : TON;  (* T3: Finishing time, product B 8 s *)
    T4_InspTimeout : TON;  (* T4: Inspection timeout 5 s *)
    T7_IndexWdg : TON;  (* T7: Index watchdog 15 s (jam / no material) *)
    T8_IdDwell : TON;  (* T8: Identification dwell 0.5 s *)
    T9_IdTimeout : TON;  (* T9: Identification timeout 3 s *)
    T10_RobotWdg : TON;  (* T10: Robot watchdog 30 s *)
    T11_PackWdg : TON;  (* T11: Packaging watchdog 15 s *)
    C0_GoodCtr : CTU;  (* C0: Good counter *)
    C1_RejectCtr : CTU;  (* C1: Reject counter *)
    C2_PackagedCtr : CTU;  (* C2: Packaged counter *)
    (* Rising-edge detectors *)
    Rt_M108 : R_TRIG;
    Rt_M102 : R_TRIG;
    Rt_X2 : R_TRIG;
    Rt_M100 : R_TRIG;
    Rt_X0 : R_TRIG;
    Rt_X14 : R_TRIG;
    Rt_X15 : R_TRIG;
    pw : BOOL;  (* power flow of the current rung *)
  END_VAR

  (* ---- rising-edge snapshot (start of scan) ---- *)
  Rt_M108(CLK := M108_HmiEStop);
  Rt_M102(CLK := M102_HmiReset);
  Rt_X2(CLK := X2_ResetPB);
  Rt_M100(CLK := M100_HmiStart);
  Rt_X0(CLK := X0_StartPB);
  Rt_X14(CLK := X14_RobotPlaceDone);
  Rt_X15(CLK := X15_PackComplete);

  (* ================ A  Faults, emergency stop and reset ================ *)

  (* R1: Robot fault or robot timeout *)
  pw := (NOT X16_RobotHealthy OR T10_RobotWdg.Q);
  IF pw THEN M1_Fault := TRUE; END_IF;
  IF pw THEN D108_FaultCode := 2; END_IF;

  (* R2: Inspection timeout (no vision result) *)
  pw := T4_InspTimeout.Q;
  IF pw THEN M1_Fault := TRUE; END_IF;
  IF pw THEN D108_FaultCode := 1; END_IF;

  (* R3: Index or packaging timeout (jam, no material) *)
  pw := (T7_IndexWdg.Q OR T11_PackWdg.Q);
  IF pw THEN M1_Fault := TRUE; END_IF;
  IF pw THEN D108_FaultCode := 3; END_IF;

  (* R4: Unknown product (identification failed) *)
  pw := T9_IdTimeout.Q;
  IF pw THEN M1_Fault := TRUE; END_IF;
  IF pw THEN D108_FaultCode := 4; END_IF;

  (* R5: Emergency stop (chain open or HMI button) *)
  pw := (NOT X17_EStopOK OR Rt_M108.Q OR M2_EStop);
  IF pw THEN M2_EStop := TRUE; END_IF;
  IF pw THEN D108_FaultCode := 5; END_IF;

  (* R6: Reset: clear faults, E-stop and the line *)
  pw := (Rt_M102.Q OR Rt_X2.Q) AND
    X16_RobotHealthy AND
    X17_EStopOK;
  IF pw THEN M1_Fault := FALSE; M2_EStop := FALSE; END_IF;
  IF pw THEN M3_PhaseIndex := FALSE; M4_PhaseWork := FALSE; END_IF;
  IF pw THEN M50_S1_Occupied := FALSE; M51_S2_Occupied := FALSE; M52_S3_Occupied := FALSE; M53_S4_Occupied := FALSE; M54_S5_Occupied := FALSE; M55_DefectAtS4 := FALSE; M56_DefectAtS5 := FALSE; M60_S1_IsTypeB := FALSE; M61_S2_IsTypeB := FALSE; M62_S3_IsTypeB := FALSE; M63_S4_IsTypeB := FALSE; M64_S5_IsTypeB := FALSE; M69_FeedDone := FALSE; M70_S1_IdDone := FALSE; M71_S2_CutDone := FALSE; M72_S3_FinishDone := FALSE; M73_S4_InspDone := FALSE; M75_PackBusy := FALSE; M80_S1_Departed := FALSE; M81_S2_Departed := FALSE; M82_S3_Departed := FALSE; M83_S4_Departed := FALSE; M84_S5_Departed := FALSE; END_IF;
  IF pw THEN D107_Phase := 0; END_IF;
  IF pw THEN D108_FaultCode := 0; END_IF;
  IF pw THEN D110_CycleMs := 0; END_IF;

  (* ================ B  System start / stop ================ *)

  (* R7: Start system *)
  pw := (Rt_M100.Q OR Rt_X0.Q) AND
    NOT M1_Fault AND
    NOT M2_EStop AND
    X16_RobotHealthy AND
    X17_EStopOK;
  IF pw THEN M0_SystemRun := TRUE; END_IF;

  (* R8: Stop system (stop dominates) *)
  pw := (M101_HmiStop OR X1_StopPB OR NOT X16_RobotHealthy OR NOT X17_EStopOK OR M1_Fault OR M2_EStop);
  IF pw THEN M0_SystemRun := FALSE; END_IF;

  (* ================ C  Mode and product selection ================ *)

  (* R9: Select auto mode *)
  pw := M103_HmiAuto;
  IF pw THEN M31_ManualMode := FALSE; END_IF;

  (* R10: Select manual mode *)
  pw := M104_HmiManual;
  IF pw THEN M31_ManualMode := TRUE; END_IF;

  (* R11: Manual override: product A *)
  pw := M105_HmiProdA AND
    M31_ManualMode;
  IF pw THEN M17_ManualTypeB := FALSE; END_IF;
  IF pw THEN D106_CurrentProduct := 1; END_IF;

  (* R12: Manual override: product B *)
  pw := M106_HmiProdB AND
    M31_ManualMode;
  IF pw THEN M17_ManualTypeB := TRUE; END_IF;
  IF pw THEN D106_CurrentProduct := 2; END_IF;

  (* ================ D  Line control: indexing and shifting ================ *)

  (* R13: Idle start: feed the first product *)
  pw := M0_SystemRun AND
    NOT M3_PhaseIndex AND
    NOT M4_PhaseWork AND
    NOT M1_Fault AND
    NOT M2_EStop;
  IF pw THEN M4_PhaseWork := TRUE; END_IF;
  IF pw THEN D107_Phase := 20; END_IF;

  (* R14: Every old product has left its station *)
  M28_AllDeparted := M0_SystemRun AND
    M3_PhaseIndex AND
    (NOT M50_S1_Occupied OR M80_S1_Departed) AND
    (NOT M51_S2_Occupied OR M81_S2_Departed) AND
    (NOT M52_S3_Occupied OR M82_S3_Departed) AND
    (NOT M53_S4_Occupied OR M83_S4_Departed) AND
    (NOT M54_S5_Occupied OR M84_S5_Departed);

  (* R15: Every expected product has arrived *)
  M29_AllArrived := M0_SystemRun AND
    M3_PhaseIndex AND
    X3_AtS1_FeedId AND
    (NOT M50_S1_Occupied OR X6_AtS2_Cutting) AND
    (NOT M51_S2_Occupied OR X7_AtS3_Finishing) AND
    (NOT M52_S3_Occupied OR X10_AtS4_Inspection) AND
    (NOT M53_S4_Occupied OR X20_AtS5_Robot);

  (* R16: Index complete *)
  M24_IndexComplete := M28_AllDeparted AND
    M29_AllArrived;

  (* R17: Departure latch S1 *)
  pw := M0_SystemRun AND
    M3_PhaseIndex AND
    M50_S1_Occupied AND
    NOT X3_AtS1_FeedId;
  IF pw THEN M80_S1_Departed := TRUE; END_IF;

  (* R18: Departure latch S2 *)
  pw := M0_SystemRun AND
    M3_PhaseIndex AND
    M51_S2_Occupied AND
    NOT X6_AtS2_Cutting;
  IF pw THEN M81_S2_Departed := TRUE; END_IF;

  (* R19: Departure latch S3 *)
  pw := M0_SystemRun AND
    M3_PhaseIndex AND
    M52_S3_Occupied AND
    NOT X7_AtS3_Finishing;
  IF pw THEN M82_S3_Departed := TRUE; END_IF;

  (* R20: Departure latch S4 *)
  pw := M0_SystemRun AND
    M3_PhaseIndex AND
    M53_S4_Occupied AND
    NOT X10_AtS4_Inspection;
  IF pw THEN M83_S4_Departed := TRUE; END_IF;

  (* R21: Departure latch S5 *)
  pw := M0_SystemRun AND
    M3_PhaseIndex AND
    M54_S5_Occupied AND
    NOT X20_AtS5_Robot;
  IF pw THEN M84_S5_Departed := TRUE; END_IF;

  (* R22: Shift occupancy one station forward *)
  pw := M24_IndexComplete;
  IF pw THEN M54_S5_Occupied := M53_S4_Occupied; END_IF;
  IF pw THEN M53_S4_Occupied := M52_S3_Occupied; END_IF;
  IF pw THEN M52_S3_Occupied := M51_S2_Occupied; END_IF;
  IF pw THEN M51_S2_Occupied := M50_S1_Occupied; END_IF;
  IF pw THEN M50_S1_Occupied := TRUE; END_IF;

  (* R23: Shift product types one station forward *)
  pw := M24_IndexComplete;
  IF pw THEN M64_S5_IsTypeB := M63_S4_IsTypeB; END_IF;
  IF pw THEN M63_S4_IsTypeB := M62_S3_IsTypeB; END_IF;
  IF pw THEN M62_S3_IsTypeB := M61_S2_IsTypeB; END_IF;
  IF pw THEN M61_S2_IsTypeB := M60_S1_IsTypeB; END_IF;
  IF pw THEN M60_S1_IsTypeB := FALSE; END_IF;

  (* R24: Shift defect flag from station 4 to station 5 *)
  pw := M24_IndexComplete;
  IF pw THEN M56_DefectAtS5 := M55_DefectAtS4; END_IF;
  IF pw THEN M55_DefectAtS4 := FALSE; END_IF;

  (* R25: Clear done flags and departure latches *)
  pw := M24_IndexComplete;
  IF pw THEN M69_FeedDone := FALSE; M70_S1_IdDone := FALSE; M71_S2_CutDone := FALSE; M72_S3_FinishDone := FALSE; M73_S4_InspDone := FALSE; END_IF;
  IF pw THEN M80_S1_Departed := FALSE; M81_S2_Departed := FALSE; M82_S3_Departed := FALSE; M83_S4_Departed := FALSE; M84_S5_Departed := FALSE; END_IF;

  (* R26: Index to work *)
  pw := M24_IndexComplete;
  IF pw THEN M3_PhaseIndex := FALSE; END_IF;
  IF pw THEN M4_PhaseWork := TRUE; END_IF;
  IF pw THEN D107_Phase := 20; END_IF;

  (* ================ E  Station 0: Feeder ================ *)

  (* R27: Feeder at the start of the belt *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    NOT M69_FeedDone;
  Y0_FeederMotor := pw;
  T0_FeedTime(IN := pw, PT := T#2000ms);

  (* R28: Feed done *)
  pw := T0_FeedTime.Q;
  IF pw THEN M69_FeedDone := TRUE; END_IF;

  (* ================ F  Station 1: Identification ================ *)

  (* R29: Identification dwell *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    M50_S1_Occupied AND
    X3_AtS1_FeedId;
  T8_IdDwell(IN := pw, PT := T#500ms);

  (* R30: Identification timeout *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    M50_S1_Occupied AND
    X3_AtS1_FeedId AND
    NOT M70_S1_IdDone;
  T9_IdTimeout(IN := pw, PT := T#3000ms);

  (* R31: Identify product A (auto mode) *)
  pw := T8_IdDwell.Q AND
    NOT M31_ManualMode AND
    X4_TypeA_Sensor AND
    NOT X5_TypeB_Sensor;
  IF pw THEN M60_S1_IsTypeB := FALSE; END_IF;
  IF pw THEN D106_CurrentProduct := 1; END_IF;

  (* R32: Identify product B (auto mode) *)
  pw := T8_IdDwell.Q AND
    NOT M31_ManualMode AND
    X5_TypeB_Sensor AND
    NOT X4_TypeA_Sensor;
  IF pw THEN M60_S1_IsTypeB := TRUE; END_IF;
  IF pw THEN D106_CurrentProduct := 2; END_IF;

  (* R33: Product type from HMI (manual mode) *)
  pw := T8_IdDwell.Q AND
    M31_ManualMode;
  IF pw THEN M60_S1_IsTypeB := M17_ManualTypeB; END_IF;

  (* R34: Identification done *)
  pw := T8_IdDwell.Q AND
    (M31_ManualMode OR (X4_TypeA_Sensor AND NOT X5_TypeB_Sensor) OR (NOT X4_TypeA_Sensor AND X5_TypeB_Sensor));
  IF pw THEN M70_S1_IdDone := TRUE; END_IF;

  (* ================ G  Station 2: Cutting (stage 1) ================ *)

  (* R35: Cutter and cutting time *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    M51_S2_Occupied AND
    X6_AtS2_Cutting AND
    NOT M71_S2_CutDone;
  Y2_Cutter := pw;
  T1_CutTime(IN := pw, PT := T#3000ms);

  (* R36: Cutting done *)
  pw := T1_CutTime.Q;
  IF pw THEN M71_S2_CutDone := TRUE; END_IF;

  (* ================ H  Station 3: Finishing (stage 2) ================ *)

  (* R37: Heat press (single coil) *)
  Y3_HeatPress := M0_SystemRun AND
    M4_PhaseWork AND
    M52_S3_Occupied AND
    X7_AtS3_Finishing AND
    NOT M72_S3_FinishDone;

  (* R38: Finishing time, product A (5 s) *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    M52_S3_Occupied AND
    X7_AtS3_Finishing AND
    NOT M72_S3_FinishDone AND
    NOT M62_S3_IsTypeB;
  T2_FinishA(IN := pw, PT := T#5000ms);

  (* R39: Finishing time, product B (8 s) *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    M52_S3_Occupied AND
    X7_AtS3_Finishing AND
    NOT M72_S3_FinishDone AND
    M62_S3_IsTypeB;
  T3_FinishB(IN := pw, PT := T#8000ms);

  (* R40: Finishing done *)
  pw := (T2_FinishA.Q OR T3_FinishB.Q);
  IF pw THEN M72_S3_FinishDone := TRUE; END_IF;

  (* ================ I  Station 4: Quality inspection ================ *)

  (* R41: Inspection trigger (camera) *)
  Y4_InspTrigger := M0_SystemRun AND
    M4_PhaseWork AND
    M53_S4_Occupied AND
    X10_AtS4_Inspection AND
    NOT M73_S4_InspDone;

  (* R42: Inspection timeout timer *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    M53_S4_Occupied AND
    X10_AtS4_Inspection AND
    NOT M73_S4_InspDone AND
    NOT X11_InspComplete;
  T4_InspTimeout(IN := pw, PT := T#5000ms);

  (* R43: Result: good product *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    M53_S4_Occupied AND
    X10_AtS4_Inspection AND
    X11_InspComplete AND
    NOT X12_DefectDetected;
  IF pw THEN M55_DefectAtS4 := FALSE; END_IF;

  (* R44: Result: defective product *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    M53_S4_Occupied AND
    X10_AtS4_Inspection AND
    X11_InspComplete AND
    X12_DefectDetected;
  IF pw THEN M55_DefectAtS4 := TRUE; END_IF;

  (* R45: Inspection done *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    M53_S4_Occupied AND
    X10_AtS4_Inspection AND
    X11_InspComplete;
  IF pw THEN M73_S4_InspDone := TRUE; END_IF;

  (* ================ J  Station 5: Robot sorting ================ *)

  (* R46: Robot pick request *)
  Y5_RobotPickReq := M0_SystemRun AND
    M4_PhaseWork AND
    M54_S5_Occupied AND
    X20_AtS5_Robot AND
    NOT X14_RobotPlaceDone AND
    (M56_DefectAtS5 OR NOT M75_PackBusy);

  (* R47: Destination: good *)
  Y6_DestGood := M0_SystemRun AND
    M4_PhaseWork AND
    M54_S5_Occupied AND
    NOT M56_DefectAtS5;

  (* R48: Destination: reject *)
  Y7_DestReject := M0_SystemRun AND
    M4_PhaseWork AND
    M54_S5_Occupied AND
    M56_DefectAtS5;

  (* R49: Robot watchdog timer *)
  pw := M0_SystemRun AND
    M4_PhaseWork AND
    M54_S5_Occupied;
  T10_RobotWdg(IN := pw, PT := T#30000ms);

  (* R50: Good product sorted (pulse) *)
  M22_GoodSorted := M0_SystemRun AND
    M4_PhaseWork AND
    M54_S5_Occupied AND
    NOT M56_DefectAtS5 AND
    Rt_X14.Q;

  (* R51: Reject product sorted (pulse) *)
  M23_RejectSorted := M0_SystemRun AND
    M4_PhaseWork AND
    M54_S5_Occupied AND
    M56_DefectAtS5 AND
    Rt_X14.Q;

  (* R52: Product has left station 5 *)
  pw := (M22_GoodSorted OR M23_RejectSorted);
  IF pw THEN M54_S5_Occupied := FALSE; END_IF;

  (* ================ K  Packaging machine ================ *)

  (* R53: Start packaging *)
  pw := M22_GoodSorted;
  IF pw THEN M75_PackBusy := TRUE; END_IF;

  (* R54: Packaging machine and watchdog *)
  pw := M0_SystemRun AND
    M75_PackBusy;
  Y10_PackMachine := pw;
  T11_PackWdg(IN := pw, PT := T#15000ms);

  (* R55: Packaged (pulse) *)
  M32_PackagedPulse := M0_SystemRun AND
    M75_PackBusy AND
    Rt_X15.Q;

  (* R56: Packaging done *)
  pw := M32_PackagedPulse;
  IF pw THEN M75_PackBusy := FALSE; END_IF;

  (* ================ L  Work complete, conveyor and watchdog ================ *)

  (* R57: Every occupied station is finished *)
  M26_StationsDone := (NOT M50_S1_Occupied OR M70_S1_IdDone) AND
    (NOT M51_S2_Occupied OR M71_S2_CutDone) AND
    (NOT M52_S3_Occupied OR M72_S3_FinishDone) AND
    (NOT M53_S4_Occupied OR M73_S4_InspDone) AND
    NOT M54_S5_Occupied;

  (* R58: Work complete (feed done, robot home) *)
  M25_WorkComplete := M0_SystemRun AND
    M4_PhaseWork AND
    M69_FeedDone AND
    M26_StationsDone AND
    X13_RobotReady;

  (* R59: Work to index *)
  pw := M25_WorkComplete;
  IF pw THEN M4_PhaseWork := FALSE; END_IF;
  IF pw THEN M3_PhaseIndex := TRUE; END_IF;
  IF pw THEN D107_Phase := 10; END_IF;

  (* R60: Main conveyor (single coil) *)
  Y1_MainConveyor := M0_SystemRun AND
    M3_PhaseIndex AND
    NOT M24_IndexComplete;

  (* R61: Index watchdog (15 s) *)
  pw := M0_SystemRun AND
    M3_PhaseIndex;
  T7_IndexWdg(IN := pw, PT := T#15000ms);

  (* ================ M  Counters, takt time and KPIs ================ *)

  (* R62: Stopwatches (add one scan = 20 ms) *)
  pw := M0_SystemRun AND
    (M3_PhaseIndex OR M4_PhaseWork);
  IF pw THEN D110_CycleMs := (D110_CycleMs + 20); END_IF;
  IF pw THEN D111_RunMs := (D111_RunMs + 20); END_IF;

  (* R63: Takt time of the last product (0.1 s) *)
  pw := (M22_GoodSorted OR M23_RejectSorted);
  IF pw THEN D104_TaktTime01s := DINT_TO_INT((D110_CycleMs / 100)); END_IF;
  IF pw THEN D110_CycleMs := 0; END_IF;

  (* R64: Good counter *)
  pw := M22_GoodSorted;
  C0_GoodCtr(CU := pw, R := M107_HmiClear, PV := 9999);

  (* R65: Reject counter *)
  pw := M23_RejectSorted;
  C1_RejectCtr(CU := pw, R := M107_HmiClear, PV := 9999);

  (* R66: Packaged counter *)
  pw := M32_PackagedPulse;
  C2_PackagedCtr(CU := pw, R := M107_HmiClear, PV := 9999);

  (* R67: Copy counters to data registers *)
  pw := TRUE;
  IF pw THEN D100_GoodCount := C0_GoodCtr.CV; END_IF;
  IF pw THEN D101_RejectCount := C1_RejectCtr.CV; END_IF;
  IF pw THEN D109_PackagedCount := C2_PackagedCtr.CV; END_IF;

  (* R68: Total count *)
  pw := TRUE;
  IF pw THEN D102_TotalCount := DINT_TO_INT((INT_TO_DINT(D100_GoodCount) + INT_TO_DINT(D101_RejectCount))); END_IF;

  (* R69: Defect percentage x100 *)
  pw := (D102_TotalCount > 0);
  IF pw THEN D103_DefectPctX100 := DINT_TO_INT(((INT_TO_DINT(D101_RejectCount) * 10000) / INT_TO_DINT(D102_TotalCount))); END_IF;

  (* R70: Production rate: average products per minute x10 *)
  pw := (D111_RunMs >= 100);
  IF pw THEN D105_RateX10PerMin := DINT_TO_INT(((INT_TO_DINT(D102_TotalCount) * 6000) / (D111_RunMs / 100))); END_IF;

  (* R71: Clear counters and KPIs (HMI) *)
  pw := M107_HmiClear;
  IF pw THEN D103_DefectPctX100 := 0; END_IF;
  IF pw THEN D104_TaktTime01s := 0; END_IF;
  IF pw THEN D105_RateX10PerMin := 0; END_IF;
  IF pw THEN D110_CycleMs := 0; END_IF;
  IF pw THEN D111_RunMs := 0; END_IF;

  (* ================ N  Indicators ================ *)

  (* R72: Run lamp *)
  Y12_RunLamp := M0_SystemRun;

  (* R73: Alarm buzzer and fault lamp *)
  pw := (M1_Fault OR M2_EStop);
  Y11_AlarmBuzzer := pw;
  Y13_FaultLamp := pw;

END_PROGRAM


CONFIGURATION Config0

  RESOURCE Res0 ON PLC
    TASK TaskMain(INTERVAL := T#20ms,PRIORITY := 0);
    PROGRAM Inst0 WITH TaskMain : SmartTextile;
  END_RESOURCE
END_CONFIGURATION
