# Ladder: rung map

Editor rung numbers in program SmartTextileLadder (top to bottom). A rung with two numbers is a condition rung followed by an output rung, so the power flow is evaluated once per scan.

| Original | Section | Rung | Editor rung(s) | Flag bit read by the data program |
|---|---|---|---|---|
| R1 | A | Robot fault or robot timeout | 1 | F_R01_RobotFaultRobot |
| R2 | A | Inspection timeout (no vision result) | 2 | F_R02_InspectionTimeoutVision |
| R3 | A | Index or packaging timeout (jam, no material) | 3 | F_R03_IndexPackagingTimeout |
| R4 | A | Unknown product (identification failed) | 4 | F_R04_UnknownProductIdentification |
| R5 | A | Emergency stop (chain open or HMI button) | 5 + 6 | F_R05_EmergencyStopChain |
| R6 | A | Reset: clear faults, E-stop and the line | 7 + 8 | F_R06_ResetClearFaults |
| R7 | B | Start system | 9 |  |
| R8 | B | Stop system (stop dominates) | 10 |  |
| R9 | C | Select auto mode | 11 |  |
| R10 | C | Select manual mode | 12 |  |
| R11 | C | Manual override: product A | 13 | F_R11_ManualOverrideProduct |
| R12 | C | Manual override: product B | 14 | F_R12_ManualOverrideProduct |
| R13 | D | Idle start: feed the first product | 15 + 16 | F_R13_IdleStartFeed |
| R14 | D | Every old product has left its station | 17 |  |
| R15 | D | Every expected product has arrived | 18 |  |
| R16 | D | Index complete | 19 |  |
| R17 | D | Departure latch S1 | 20 |  |
| R18 | D | Departure latch S2 | 21 |  |
| R19 | D | Departure latch S3 | 22 |  |
| R20 | D | Departure latch S4 | 23 |  |
| R21 | D | Departure latch S5 | 24 |  |
| R22 | D | Shift occupancy one station forward | 25 |  |
| R23 | D | Shift product types one station forward | 26 |  |
| R24 | D | Shift defect flag from station 4 to station 5 | 27 |  |
| R25 | D | Clear done flags and departure latches | 28 |  |
| R26 | D | Index to work | 29 | F_R26_IndexWork |
| R27 | E | Feeder at the start of the belt | 30 | F_R27_FeederStartBelt |
| R28 | E | Feed done | 31 |  |
| R29 | F | Identification dwell | 32 | F_R29_IdentificationDwell |
| R30 | F | Identification timeout | 33 | F_R30_IdentificationTimeout |
| R31 | F | Identify product A (auto mode) | 34 | F_R31_IdentifyProductAuto |
| R32 | F | Identify product B (auto mode) | 35 | F_R32_IdentifyProductB |
| R33 | F | Product type from HMI (manual mode) | 36 |  |
| R34 | F | Identification done | 37 |  |
| R35 | G | Cutter and cutting time | 38 | F_R35_CutterCuttingTime |
| R36 | G | Cutting done | 39 |  |
| R37 | H | Heat press (single coil) | 40 |  |
| R38 | H | Finishing time, product A (5 s) | 41 | F_R38_FinishingTimeProduct |
| R39 | H | Finishing time, product B (8 s) | 42 | F_R39_FinishingTimeProduct |
| R40 | H | Finishing done | 43 |  |
| R41 | I | Inspection trigger (camera) | 44 |  |
| R42 | I | Inspection timeout timer | 45 | F_R42_InspectionTimeoutTimer |
| R43 | I | Result: good product | 46 |  |
| R44 | I | Result: defective product | 47 |  |
| R45 | I | Inspection done | 48 |  |
| R46 | J | Robot pick request | 49 |  |
| R47 | J | Destination: good | 50 |  |
| R48 | J | Destination: reject | 51 |  |
| R49 | J | Robot watchdog timer | 52 | F_R49_RobotWatchdogTimer |
| R50 | J | Good product sorted (pulse) | 53 |  |
| R51 | J | Reject product sorted (pulse) | 54 |  |
| R52 | J | Product has left station 5 | 55 |  |
| R53 | K | Start packaging | 56 |  |
| R54 | K | Packaging machine and watchdog | 57 | F_R54_PackagingMachineWatchdog |
| R55 | K | Packaged (pulse) | 58 |  |
| R56 | K | Packaging done | 59 |  |
| R57 | L | Every occupied station is finished | 60 |  |
| R58 | L | Work complete (feed done, robot home) | 61 |  |
| R59 | L | Work to index | 62 | F_R59_WorkIndex |
| R60 | L | Main conveyor (single coil) | 63 |  |
| R61 | L | Index watchdog (15 s) | 64 | F_R61_IndexWatchdog15 |
| R62 | M | Stopwatches (add one scan = 20 ms) | 65 | F_R62_StopwatchesAddOne |
| R63 | M | Takt time of the last product (0.1 s) | 66 | F_R63_TaktTimeLast |
| R64 | M | Good counter | 67 | F_R64_GoodCounter |
| R65 | M | Reject counter | 68 | F_R65_RejectCounter |
| R66 | M | Packaged counter | 69 | F_R66_PackagedCounter |
| R71 | M | Clear counters and KPIs (HMI) | 70 | F_R71_ClearCountersKPIs |
| R72 | N | Run lamp | 71 |  |
| R73 | N | Alarm buzzer and fault lamp | 72 |  |

Not drawn as ladder (arithmetic only, in SmartTextileData): R67 copy counters, R68 total count, R69 defect %, R70 production rate.
