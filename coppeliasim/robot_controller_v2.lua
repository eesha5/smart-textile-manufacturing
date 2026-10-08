sim = require('sim')

--------------------------------------------------------------
-- TUNING
--------------------------------------------------------------
DEFECT_EVERY_N = 3      -- every 3rd product is Product_B (set 2 for alternating A/B)
LIFT           = 0.15   -- clearance above a pick / place point
ARM_MIN_Z      = 0.18   -- lowest gripper height at the good bin
MAX_DROP       = 0.12   -- gripper goes down to within this height of the slot before release
SETTLE_GAP     = 0.002  -- tiny gap so sheets do not intersect
PACK_LEVELS    = 4      -- sheets per box before the box is "shipped"
REJECT_LEVELS  = 6
KEEP_REJECTS   = true   -- rejected sheets pile up in the reject bin
PARK           = {0.0, 6.0, -6.0}   -- where the used product object waits (out of sight)
ID_TIMEOUT     = 3.0    -- seconds State 1 waits for the ID sensor before using the fed product
SPEED_FACTOR   = 2.0    -- 1.0 = original speed, 2.0 = twice as fast (all robot / belt / packing motions)
WATCHDOG_S     = 25.0   -- any state blocked this long triggers the safety reset (State 8)

-- process times, identical to the PLC program (SmartTextile)
CONV_T         = 3.0    -- belt travel from the feeder to the pick position
CUT_T          = 3.0    -- processing stage 1 (cutting)
FIN_A_T        = 5.0    -- processing stage 2 (finishing), product A
FIN_B_T        = 8.0    -- processing stage 2 (finishing), product B
INSP_MOVE_T    = 0.8    -- product moves to the inspection station
INSP_READ_T    = 2.0    -- earliest moment the OpenCV result is read (after arrival + vision frames)
INSP_TIMEOUT_T = 9.0    -- fall back to the product class if OpenCV does not answer
SORT_T         = 3.0    -- product travels to its bin

FEED_POS = {-1.650, 0, 0.30}


--------------------------------------------------------------
-- HELPERS
--------------------------------------------------------------
function clamp01(t)
    if t < 0 then return 0 elseif t > 1 then return 1 end
    return t
end

function ease(t)
    t = clamp01(t)
    return t * t * (3 - 2 * t)
end

function lerp3(a, b, t)
    local e = ease(t)
    return {a[1] + (b[1] - a[1]) * e,
            a[2] + (b[2] - a[2]) * e,
            a[3] + (b[3] - a[3]) * e}
end

function lerpLin(a, b, t)
    t = clamp01(t)
    return {a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t,
            a[3] + (b[3] - a[3]) * t}
end

function seg(t, t0, t1)
    return (t - t0) / (t1 - t0)
end

-- half extent of an object along world Z (works for rotated boxes too)
function halfZ(h, fallback)
    local ok, s = pcall(sim.getShapeBB, h)
    if not ok or s == nil then return fallback end
    local m = sim.getObjectMatrix(h, -1)
    return (math.abs(m[9]) * s[1] + math.abs(m[10]) * s[2] + math.abs(m[11]) * s[3]) / 2
end

function topOf(h)
    local p = sim.getObjectPosition(h, -1)
    return p[3] + halfZ(h, 0.05)
end

function removeAll(handles)
    if handles == nil then return end
    local ok = pcall(sim.removeObjects, handles)
    if not ok then
        for _, h in ipairs(handles) do pcall(sim.removeObject, h) end
    end
end

-- permanent copy of a product (with its stains) placed at pos
function clonePersistent(src, pos, alias)
    local ok, tree = pcall(sim.getObjectsInTree, src, sim.handle_all, 0)
    if not ok or tree == nil or #tree == 0 then tree = {src} end
    local ok2, copies = pcall(sim.copyPasteObjects, tree, 0)
    if not ok2 or copies == nil or #copies == 0 then return nil end

    local isCopy = {}
    for _, h in ipairs(copies) do isCopy[h] = true end
    local root = copies[1]
    for _, h in ipairs(copies) do
        local p = sim.getObjectParent(h)
        if p == -1 or not isCopy[p] then
            root = h
            break
        end
    end

    sim.setObjectPosition(root, -1, pos)
    for _, h in ipairs(copies) do
        pcall(sim.setObjectInt32Param, h, sim.shapeintparam_static, 1)
        pcall(sim.setObjectInt32Param, h, sim.shapeintparam_respondable, 0)
        pcall(sim.setObjectSpecialProperty, h, sim.objectspecialproperty_renderable)
    end
    pcall(sim.setObjectAlias, root, alias)
    return copies
end

function stackRestZ(s, level)
    return topOf(s.bin) + SETTLE_GAP + prodHalf + (level - 1) * prodThick
end

function stackNextLevel(s)
    if #s.items >= s.cap then return 1, true end
    return #s.items + 1, false
end

function stackClear(s)
    for _, copies in ipairs(s.items) do removeAll(copies) end
    s.items = {}
    s.shipped = s.shipped + 1
end

function stackCommit(s, product, pos, alias)
    local copies = clonePersistent(product, pos, alias)
    if copies ~= nil then
        table.insert(s.items, copies)
        sim.setObjectPosition(product, -1, PARK)       -- the original is free for the next cycle
    else
        sim.setObjectPosition(product, -1, pos)        -- fallback: the product itself rests on the slot
    end
end

-- never let a stacking problem kill the controller
function safeCommit(s, product, pos, alias)
    local ok, err = pcall(stackCommit, s, product, pos, alias)
    if not ok then
        sim.addLog(sim.verbosity_scripterrors, 'STACK COMMIT FAILED: ' .. tostring(err))
        pcall(sim.setObjectPosition, product, -1, pos)
    end
end

function safeClear(s)
    local ok, err = pcall(stackClear, s)
    if not ok then
        sim.addLog(sim.verbosity_scripterrors, 'STACK CLEAR FAILED: ' .. tostring(err))
        s.items = {}
    end
end

function publishCounts()
    sim.setInt32Signal('GoodCount', goodCount)
    sim.setInt32Signal('DefectCount', rejectCount)
    sim.setInt32Signal('TotalCount', totalCount)
    sim.setInt32Signal('PackagedCount', packagedCount)
    sim.setInt32Signal('BoxesShipped', stacks.pack.shipped)
end


--------------------------------------------------------------
-- INIT
--------------------------------------------------------------
function sysCall_init()

    target = sim.getObject('/IRB140/link1_visible/manipulationSphereBase/manipulationSphere/target')
    graspPoint = sim.getObject('/IRB140/**/Robot_GraspPoint')

    pickApproach = sim.getObject('/Robot_PickApproach')
    pickTarget = sim.getObject('/Robot_PickTarget')
    goodTarget = sim.getObject('/Robot_GoodTarget')
    rejectTarget = sim.getObject('/Robot_RejectTarget')
    packagingTarget = sim.getObject('/Robot_PackagingTarget')

    productA = sim.getObject('/Product_A')
    productB = sim.getObject('/Product_B')

    productIdSensor = sim.getObject('/Sensor_ProductId')
    inspectionTrigger = sim.getObject('/Sensor_InspectionTrigger')

    process1Tool = sim.getObject('/Process1_Tool')
    process2Press = sim.getObject('/Process2_Press')

    goodBin = sim.getObject('/Good_Bin')
    rejectBin = sim.getObject('/Reject_Bin')
    packBase = sim.getObject('/Packaging_Base')

    process1Base = sim.getObjectPosition(process1Tool, -1)
    process2Base = sim.getObjectPosition(process2Press, -1)

    prodHalf = halfZ(productA, 0.025)
    prodThick = 2 * prodHalf

    stacks = {
        pack   = {items = {}, bin = packBase,  cap = PACK_LEVELS,   shipped = 0},
        reject = {items = {}, bin = rejectBin, cap = REJECT_LEVELS, shipped = 0},
    }

    cycleCount = 0
    currentProduct = nil
    isDefective = false

    state = 0
    timer = 0
    carrying = false

    lastState = -1
    stateSince = 0
    lastSeen = nil

    p5, p6, p7, p9 = nil, nil, nil, nil

    goodCount = 0
    rejectCount = 0
    totalCount = 0
    packagedCount = 0

    sim.clearFloatSignal('InspectionResult')
    sim.setInt32Signal('InspectionActive', 0)
    sim.setInt32Signal('RobotState', 0)
    publishCounts()

    sim.addLog(sim.verbosity_scriptinfos, '================================')
    sim.addLog(sim.verbosity_scriptinfos, 'SMART TEXTILE SYSTEM STARTED (controller v2)')
    sim.addLog(sim.verbosity_scriptinfos, 'OPENCV INSPECTION ENABLED')
    sim.addLog(sim.verbosity_scriptinfos, string.format(
        'SHEET THICKNESS %.3f | GOOD BIN TOP z=%.3f | REJECT BIN TOP z=%.3f | PACKAGING BASE TOP z=%.3f',
        prodThick, topOf(goodBin), topOf(rejectBin), topOf(packBase)))
    sim.addLog(sim.verbosity_scriptinfos, '================================')

end


--------------------------------------------------------------
-- MAIN
--------------------------------------------------------------
function sysCall_actuation()

    --------------------------------------------------
    -- PLC PERMISSIVE: the PLC (or the E-stop on the HMI) can hold the cell.
    -- If no bridge is running the signal does not exist and the cell runs freely.
    --------------------------------------------------
    local plcRun = sim.getInt32Signal('PlcRun')
    if plcRun ~= nil and plcRun == 0 then
        sim.setInt32Signal('RobotHeld', 1)
        stateSince = stateSince + sim.getSimulationTimeStep()      -- do not let the watchdog fire while held
        return
    end
    sim.setInt32Signal('RobotHeld', 0)

    timer = timer + sim.getSimulationTimeStep() * SPEED_FACTOR

    sim.setInt32Signal('RobotState', state)

    --------------------------------------------------
    -- WATCHDOG: the robot can never stay blocked
    --------------------------------------------------
    local now = sim.getSimulationTime()
    if state ~= lastState then
        lastState = state
        stateSince = now
    end
    if state ~= 8 and (now - stateSince) > WATCHDOG_S then
        sim.addLog(sim.verbosity_scriptinfos, string.format(
            'WATCHDOG: state %d blocked for %.0f s - safety reset', state, now - stateSince))
        sim.setInt32Signal('InspectionActive', 0)
        p5, p6, p7, p9 = nil, nil, nil, nil
        state = 8
    end

    --------------------------------------------------
    -- CARRY PRODUCT
    --------------------------------------------------
    if carrying == true and currentProduct ~= nil then
        local gp = sim.getObjectPosition(graspPoint, -1)
        sim.setObjectPosition(currentProduct, -1, gp)
    end

    --------------------------------------------------
    -- STATE 0 : FEED  (product appears at the start of the belt)
    --------------------------------------------------
    if state == 0 then

        cycleCount = cycleCount + 1

        if cycleCount % DEFECT_EVERY_N == 0 then
            currentProduct = productB
        else
            currentProduct = productA
        end

        sim.setObjectPosition(currentProduct, -1, FEED_POS)

        carrying = false
        timer = 0
        state = 1

        sim.addLog(sim.verbosity_scriptinfos, 'PRODUCT FED')

    end

    --------------------------------------------------
    -- STATE 1 : IDENTIFICATION (cannot block any more)
    --------------------------------------------------
    if state == 1 then

        local det = nil

        -- 1) ask the sensor about each product specifically, so labels, stains or any
        --    other detectable object in range cannot block identification
        for _, cand in ipairs({productA, productB}) do
            local ok, r = pcall(sim.checkProximitySensor, productIdSensor, cand)
            if ok and r ~= nil and r > 0 then
                det = cand
                break
            end
        end

        -- 2) original generic read as a second chance
        if det == nil then
            local result, distance, detectedPoint, detectedObject =
                sim.readProximitySensor(productIdSensor)
            if result > 0 then
                local d = detectedObject
                if d ~= productA and d ~= productB then
                    local par = sim.getObjectParent(d)
                    if par == productA or par == productB then d = par end
                end
                if d == productA or d == productB then
                    det = d
                elseif d ~= lastSeen then
                    lastSeen = d
                    local ok, name = pcall(sim.getObjectAlias, d)
                    sim.addLog(sim.verbosity_scriptinfos,
                        'ID SENSOR IS SEEING: ' .. (ok and tostring(name) or tostring(d)))
                end
            end
        end

        -- 3) never wait forever
        if det == nil and timer > ID_TIMEOUT then
            det = currentProduct
            sim.addLog(sim.verbosity_scriptinfos, 'ID SENSOR TIMEOUT - USING FED PRODUCT')
        end

        if det ~= nil then

            if det == productA then
                currentProduct = productA
                isDefective = false
                sim.addLog(sim.verbosity_scriptinfos, 'PRODUCT A -> GOOD')
            else
                currentProduct = productB
                isDefective = true
                sim.addLog(sim.verbosity_scriptinfos, 'PRODUCT B -> DEFECT')
            end

            -- the robot waits above the pick position while the belt brings the product (no teleport)
            local approachPos = sim.getObjectPosition(pickApproach, -1)
            sim.setObjectPosition(target, -1, approachPos)

            timer = 0
            state = 2

        end

    end

    --------------------------------------------------
    -- STATE 2 : BELT TRAVEL, THEN PICK
    --------------------------------------------------
    if state == 2 then

        local pickPos = sim.getObjectPosition(pickTarget, -1)

        if timer < CONV_T then
            -- the belt carries the product smoothly from the feeder to the pick position
            sim.setObjectPosition(currentProduct, -1, lerpLin(FEED_POS, pickPos, timer / CONV_T))
        else
            sim.setObjectPosition(currentProduct, -1, pickPos)
            sim.setObjectPosition(target, -1, pickPos)           -- robot descends onto the product

            if timer > CONV_T + 1.2 then

                local gp = sim.getObjectPosition(graspPoint, -1)
                sim.setObjectPosition(currentProduct, -1, gp)

                carrying = true
                timer = 0
                state = 3

                sim.addLog(sim.verbosity_scriptinfos, 'PRODUCT PICKED')

            end
        end

    end

    --------------------------------------------------
    -- STATE 3 : PROCESS 1 (cutting, 3 s like the PLC)
    --------------------------------------------------
    if state == 3 then

        local d = ease(seg(timer, 0.3, 0.9)) - ease(seg(timer, CUT_T - 0.7, CUT_T - 0.1))
        sim.setObjectPosition(process1Tool, -1,
            {process1Base[1], process1Base[2], process1Base[3] - 0.08 * d})

        if timer > CUT_T then
            sim.setObjectPosition(process1Tool, -1, process1Base)
            timer = 0
            state = 4
            sim.addLog(sim.verbosity_scriptinfos, 'PROCESS 1 COMPLETE')
        end

    end

    --------------------------------------------------
    -- STATE 4 : PROCESS 2 (finishing: A 5 s, B 8 s like the PLC)
    --------------------------------------------------
    if state == 4 then

        local fin = isDefective and FIN_B_T or FIN_A_T
        local d = ease(seg(timer, 0.5, 1.2)) - ease(seg(timer, fin - 0.8, fin - 0.1))
        sim.setObjectPosition(process2Press, -1,
            {process2Base[1], process2Base[2], process2Base[3] - 0.10 * d})

        if timer > fin then
            sim.setObjectPosition(process2Press, -1, process2Base)
            timer = 0
            state = 5
            sim.addLog(sim.verbosity_scriptinfos, 'PROCESS 2 COMPLETE')
        end

    end

    --------------------------------------------------
    -- STATE 5 : REAL VISION INSPECTION
    --------------------------------------------------
    if state == 5 then

        if p5 == nil then
            carrying = false
            local ip = sim.getObjectPosition(inspectionTrigger, -1)
            ip[3] = 0.30
            p5 = {from = sim.getObjectPosition(currentProduct, -1), to = ip, arrived = false}
        end

        if timer < INSP_MOVE_T then
            -- the product travels to the inspection station (no teleport)
            sim.setObjectPosition(currentProduct, -1, lerp3(p5.from, p5.to, timer / INSP_MOVE_T))
        else
            sim.setObjectPosition(currentProduct, -1, p5.to)
            if not p5.arrived then
                p5.arrived = true
                sim.clearFloatSignal('InspectionResult')
                sim.setInt32Signal('InspectionActive', 1)
                sim.addLog(sim.verbosity_scriptinfos, 'PRODUCT AT INSPECTION STATION')
            end
        end

        if timer > INSP_READ_T then

            local result = sim.getFloatSignal('InspectionResult')

            if result ~= nil and result >= 0 then

                totalCount = totalCount + 1

                if result >= 0.5 then
                    isDefective = true
                    rejectCount = rejectCount + 1
                    sim.addLog(sim.verbosity_scriptinfos, 'OPENCV: DEFECT DETECTED')
                    state = 6
                else
                    isDefective = false
                    goodCount = goodCount + 1
                    sim.addLog(sim.verbosity_scriptinfos, 'OPENCV: GOOD PRODUCT')
                    state = 7
                end

                sim.clearFloatSignal('InspectionResult')
                sim.setInt32Signal('InspectionActive', 0)
                publishCounts()
                p5, p6, p7, p9 = nil, nil, nil, nil
                timer = 0

            end

        end

        if state == 5 and timer > INSP_TIMEOUT_T then

            totalCount = totalCount + 1

            sim.addLog(sim.verbosity_scriptinfos, 'OPENCV TIMEOUT - USING PRODUCT CLASS')

            if isDefective == true then
                rejectCount = rejectCount + 1
                state = 6
            else
                goodCount = goodCount + 1
                state = 7
            end

            sim.setInt32Signal('InspectionActive', 0)
            publishCounts()
            p5, p6, p7, p9 = nil, nil, nil, nil
            timer = 0

        end

    end

    --------------------------------------------------
    -- STATE 6 : REJECT  (product travels to the reject bin and is stacked there)
    --------------------------------------------------
    if state == 6 then

        local rejectPos = sim.getObjectPosition(rejectTarget, -1)

        if p6 == nil then
            local s = stacks.reject
            local level, clear = stackNextLevel(s)
            local useLevel = clear and 1 or level
            p6 = {start = sim.getObjectPosition(target, -1),
                  from = sim.getObjectPosition(currentProduct, -1),
                  clear = clear,
                  to = {rejectPos[1], rejectPos[2], stackRestZ(s, KEEP_REJECTS and useLevel or 1)}}
        end

        sim.setObjectPosition(target, -1, lerp3(p6.start, rejectPos, seg(timer, 0, 2.5)))

        local e = ease(timer / SORT_T)
        local pos = lerp3(p6.from, p6.to, timer / SORT_T)
        pos[3] = pos[3] + 0.12 * math.sin(math.pi * e)
        sim.setObjectPosition(currentProduct, -1, pos)

        if timer > SORT_T then

            carrying = false

            if KEEP_REJECTS then
                local s = stacks.reject
                if p6.clear then
                    safeClear(s)
                    sim.addLog(sim.verbosity_scriptinfos, 'REJECT BIN FULL - EMPTIED')
                end
                safeCommit(s, currentProduct, p6.to, 'Rejected_' .. tostring(rejectCount))
            else
                sim.setObjectPosition(currentProduct, -1, p6.to)
            end

            sim.addLog(sim.verbosity_scriptinfos, 'SORTED -> REJECT BIN')

            currentProduct = nil
            isDefective = false
            p6 = nil
            timer = 0
            state = 0

        end

    end

    --------------------------------------------------
    -- STATE 7 : GOOD BIN  (product travels to the good bin)
    --------------------------------------------------
    if state == 7 then

        local goodPos = sim.getObjectPosition(goodTarget, -1)

        if p7 == nil then
            p7 = {start = sim.getObjectPosition(target, -1),
                  from = sim.getObjectPosition(currentProduct, -1),
                  to = {goodPos[1], goodPos[2], topOf(goodBin) + SETTLE_GAP + prodHalf}}
        end

        sim.setObjectPosition(target, -1, lerp3(p7.start, goodPos, seg(timer, 0, 2.5)))

        local e = ease(timer / SORT_T)
        local pos = lerp3(p7.from, p7.to, timer / SORT_T)
        pos[3] = pos[3] + 0.12 * math.sin(math.pi * e)
        sim.setObjectPosition(currentProduct, -1, pos)

        if timer > SORT_T then

            carrying = false
            sim.setObjectPosition(currentProduct, -1, p7.to)

            sim.addLog(sim.verbosity_scriptinfos, 'SORTED -> GOOD BIN')

            p7 = nil
            p9 = nil
            timer = 0
            state = 9

        end

    end

    --------------------------------------------------
    -- STATE 8 : SAFETY RESET
    --------------------------------------------------
    if state == 8 then

        carrying = false
        currentProduct = nil
        isDefective = false
        p5, p6, p7, p9 = nil, nil, nil, nil
        timer = 0
        state = 0

    end

    --------------------------------------------------
    -- STATE 9 : GOOD PRODUCT -> PACKAGING
    --  approach, descend, grasp (sheet lifts into gripper), lift, travel,
    --  descend, release (sheet settles on the stack), retreat
    --------------------------------------------------
    if state == 9 then

        if p9 == nil then

            local pp = sim.getObjectPosition(currentProduct, -1)
            local pk = sim.getObjectPosition(packagingTarget, -1)
            local s = stacks.pack
            local level, clear = stackNextLevel(s)
            if clear then level = 1 end
            local restZ = stackRestZ(s, level)
            local armPickZ = math.max(pp[3], ARM_MIN_Z)
            local relZ = math.max(restZ, math.min(pk[3], restZ + MAX_DROP))

            p9 = {
                start    = sim.getObjectPosition(target, -1),
                pickRest = {pp[1], pp[2], pp[3]},
                pickUp   = {pp[1], pp[2], armPickZ + LIFT},
                pick     = {pp[1], pp[2], armPickZ},
                placeUp  = {pk[1], pk[2], relZ + LIFT},
                place    = {pk[1], pk[2], relZ},
                restPos  = {pk[1], pk[2], restZ},
                level = level, clear = clear,
                picked = false, released = false, placed = false,
                dropFrom = nil,
            }

            sim.addLog(sim.verbosity_scriptinfos, string.format(
                'PACKING: slot %d/%d  sheet rests at z=%.3f  gripper releases at z=%.3f',
                level, PACK_LEVELS, restZ, relZ))

        end

        local P = p9
        local t = timer
        local gp = sim.getObjectPosition(graspPoint, -1)

        if t < 1.0 then
            sim.setObjectPosition(target, -1, lerp3(P.start, P.pickUp, seg(t, 0, 1.0)))

        elseif t < 2.0 then
            sim.setObjectPosition(target, -1, lerp3(P.pickUp, P.pick, seg(t, 1.0, 2.0)))

        elseif t < 2.5 then
            -- grasp: the sheet lifts from the bin into the gripper
            local e = ease(seg(t, 2.0, 2.5))
            sim.setObjectPosition(currentProduct, -1, {
                P.pickRest[1] + (gp[1] - P.pickRest[1]) * e,
                P.pickRest[2] + (gp[2] - P.pickRest[2]) * e,
                P.pickRest[3] + (gp[3] - P.pickRest[3]) * e})

        elseif t < 3.3 then
            if not P.picked then
                P.picked = true
                carrying = true
                sim.addLog(sim.verbosity_scriptinfos, 'GOOD PRODUCT PICKED FOR PACKAGING')
            end
            sim.setObjectPosition(target, -1, lerp3(P.pick, P.pickUp, seg(t, 2.5, 3.3)))

        elseif t < 5.0 then
            sim.setObjectPosition(target, -1, lerp3(P.pickUp, P.placeUp, seg(t, 3.3, 5.0)))

        elseif t < 6.0 then
            sim.setObjectPosition(target, -1, lerp3(P.placeUp, P.place, seg(t, 5.0, 6.0)))

        elseif t < 6.5 then
            -- release: the sheet settles onto the stack
            if not P.released then
                P.released = true
                carrying = false
                P.dropFrom = {gp[1], gp[2], gp[3]}
                if P.clear then
                    safeClear(stacks.pack)
                    sim.addLog(sim.verbosity_scriptinfos, 'PACKAGING BOX FULL - SHIPPED')
                end
            end
            local e = ease(seg(t, 6.0, 6.5))
            sim.setObjectPosition(currentProduct, -1, {
                P.dropFrom[1] + (P.restPos[1] - P.dropFrom[1]) * e,
                P.dropFrom[2] + (P.restPos[2] - P.dropFrom[2]) * e,
                P.dropFrom[3] + (P.restPos[3] - P.dropFrom[3]) * e})

        elseif t < 7.3 then
            if not P.placed then
                P.placed = true
                safeCommit(stacks.pack, currentProduct, P.restPos,
                    'Packed_' .. tostring(packagedCount + 1))
                packagedCount = packagedCount + 1
                publishCounts()
                sim.addLog(sim.verbosity_scriptinfos,
                    string.format('PRODUCT PACKAGED (box %d/%d)', P.level, PACK_LEVELS))
            end
            sim.setObjectPosition(target, -1, lerp3(P.place, P.placeUp, seg(t, 6.5, 7.3)))

        else
            currentProduct = nil
            isDefective = false
            p9 = nil
            timer = 0
            state = 0
        end

    end

end


function sysCall_cleanup()
    if stacks ~= nil then
        for _, s in pairs(stacks) do
            for _, copies in ipairs(s.items) do removeAll(copies) end
            s.items = {}
        end
    end
    sim.setInt32Signal('InspectionActive', 0)
    sim.setInt32Signal('RobotState', 0)
    sim.setInt32Signal('RobotHeld', 0)
    sim.clearFloatSignal('InspectionResult')
end
