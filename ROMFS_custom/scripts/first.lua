-- -- pitch_roll_yaw_monitor.lua
-- -- Lua script to monitor pitch, roll, yaw and warn if threshold exceeded
-- -- Also controls LED to indicate status

-- local THRESHOLD = 10.0       -- degrees
-- local CHECK_INTERVAL = 1.0   -- seconds
-- local led_state = false      -- Track LED state

-- -- Function to check if AHRS is available and ready
-- function ahrs_available()
--     if not ahrs then
--         return false
--     end
--     return ahrs:healthy()
-- end

-- -- Function to check angles and send warnings
-- function check_angles()
--     -- Check if AHRS is available
--     if not ahrs_available() then
--         gcs:send_text(6, "AHRS not ready or unhealthy")
--         return
--     end

--     -- Get Euler angles using the correct modern API
--     local roll_rad = ahrs:get_roll_rad()
--     local pitch_rad = ahrs:get_pitch_rad() 
--     local yaw_rad = ahrs:get_yaw_rad()
    
--     if not roll_rad or not pitch_rad or not yaw_rad then
--         gcs:send_text(6, "Failed to get attitude data")
--         return
--     end

--     -- Convert to degrees
--     local roll  = math.deg(roll_rad)
--     local pitch = math.deg(pitch_rad)
--     local yaw   = math.deg(yaw_rad)

--     -- Check thresholds
--     local pitch_exceeded = math.abs(pitch) > THRESHOLD
--     local roll_exceeded  = math.abs(roll) > THRESHOLD
--     local yaw_exceeded   = math.abs(yaw) > THRESHOLD
--     local any_exceeded   = pitch_exceeded or roll_exceeded or yaw_exceeded

--     -- Control LED based on status
--     if any_exceeded then
--         -- Turn LED on when threshold exceeded
--         if not led_state then
--             notify:play_tune("MFT200L8>C")  -- Warning tone
--             led_state = true
--         end
        
--         -- Send warning message
--         local msg = "WARNING: Angles exceeded - "
--         if pitch_exceeded then 
--             msg = msg .. string.format("Pitch: %.1f° ", pitch) 
--         end
--         if roll_exceeded then 
--             msg = msg .. string.format("Roll: %.1f° ", roll) 
--         end
--         if yaw_exceeded then 
--             msg = msg .. string.format("Yaw: %.1f° ", yaw) 
--         end
--         msg = msg .. "LED ON"
        
--         gcs:send_text(6, msg)  -- MAV_SEVERITY_INFO
        
--     else
--         -- Turn LED off when within limits
--         if led_state then
--             led_state = false
--         end
        
--         -- Send status message (less frequent)
--         local msg = string.format("Angles OK - Pitch: %.1f°, Roll: %.1f°, Yaw: %.1f° LED OFF", 
--                                  pitch, roll, yaw)
--         gcs:send_text(7, msg)  -- MAV_SEVERITY_DEBUG
--     end
-- end

-- -- Main update function
-- function update()
--     -- Only check when armed
--     if arming and arming:is_armed() then
--         check_angles()
--     else
--         -- Reset LED state when disarmed
--         if led_state then
--             led_state = false
--         end
--     end  
--     -- Schedule next update - MUST return function and delay
--     return update, CHECK_INTERVAL * 1000  -- Convert to milliseconds
-- end

-- -- Start the script - return the update function and initial delay
-- return update, CHECK_INTERVAL * 1000







-- -- pitch_roll_led_control_unarmed.lua
-- -- Monitors pitch, roll, yaw even when disarmed
-- -- Controls LED on MAIN OUT 1 via SERVO1
-- -- Compatible with SITL, HITL, real Pixhawk

-- local SERVO_CHANNEL = 1      -- MAIN OUT 1 = SERVO1
-- local SERVO_ON_PWM = 1900
-- local SERVO_OFF_PWM = 1100
-- local THRESHOLD = 10.0       -- degrees
-- local CHECK_INTERVAL_MS = 500
-- local led_state = false

-- function ahrs_available()
--     return ahrs and ahrs:healthy()
-- end

-- function set_led(state)
--     if state ~= led_state then
--         led_state = state
--         local pwm = state and SERVO_ON_PWM or SERVO_OFF_PWM
--         servo:set_output(SERVO_CHANNEL, pwm)
--         gcs:send_text(6, string.format("✅ LED %s (PWM=%d)", state and "ON" or "OFF", pwm))
--     end
-- end

-- function check_angles()
--     if not ahrs_available() then
--         gcs:send_text(6, "⚠️ AHRS not ready")
--         set_led(false)
--         return
--     end

--     local roll_rad = ahrs:get_roll_rad()
--     local pitch_rad = ahrs:get_pitch_rad()
--     local yaw_rad = ahrs:get_yaw_rad()

--     if not roll_rad or not pitch_rad or not yaw_rad then
--         gcs:send_text(6, "⚠️ Attitude data unavailable")
--         set_led(false)
--         return
--     end

--     local roll  = math.deg(roll_rad)
--     local pitch = math.deg(pitch_rad)
--     local yaw   = math.deg(yaw_rad)

--     local pitch_exceeded = math.abs(pitch) > THRESHOLD
--     local roll_exceeded  = math.abs(roll) > THRESHOLD
--     local yaw_exceeded   = math.abs(yaw) > THRESHOLD

--     if pitch_exceeded or roll_exceeded or yaw_exceeded then
--         set_led(true)
--         local msg = "⚠️ ALERT: "
--         if pitch_exceeded then msg = msg .. string.format("Pitch=%.1f° ", pitch) end
--         if roll_exceeded then msg = msg .. string.format("Roll=%.1f° ", roll) end
--         if yaw_exceeded then msg = msg .. string.format("Yaw=%.1f° ", yaw) end
--         gcs:send_text(6, msg)
--     else
--         set_led(false)
--         gcs:send_text(7, string.format("✅ Stable: Pitch=%.1f°, Roll=%.1f°, Yaw=%.1f°", pitch, roll, yaw))
--     end
-- end

-- function update()
--     check_angles()
--     return update, CHECK_INTERVAL_MS
-- end

-- return update, CHECK_INTERVAL_MS

























-- Monitor AUX_OUT1 (SERVO9) as relay or PWM
-- and send NAMED_FLOAT to GCS continuously

gcs:send_text(6, "✅ Lua script started successfully")

local CHANNEL = 9
local AUX1_RELAY_PIN = 50

function update()

    -- Get relay pin assignment
    local relay1_pin = param:get("RELAY1_PIN")
    local servo9_function = param:get("SERVO9_FUNCTION")
    
    if relay1_pin == AUX1_RELAY_PIN then
        -- It's configured as relay
        local relay_state = param:get("RELAY1")
        if relay_state then
            gcs:send_named_float("AUX1_MODE", 1)         -- 1 = relay mode
            gcs:send_named_float("AUX1_RELAY", relay_state)
        else
            gcs:send_text(6, "⚠️ Relay param missing")
        end

    elseif servo9_function and servo9_function > 0 then
        -- It's configured as PWM output
        local pwm = SRV_Channels:get_output_pwm(CHANNEL)
        if pwm then
            gcs:send_named_float("AUX1_MODE", 2)         -- 2 = PWM mode
            gcs:send_named_float("AUX1_PWM", pwm)
        else
            gcs:send_text(6, "⚠️ PWM read failed")
        end

    else
        -- Not configured
        gcs:send_named_float("AUX1_MODE", 0)             -- 0 = unknown
        gcs:send_text(6, "⚠️ AUX1 not configured")
    end

    return update, 1000 -- run again in 1 second
end

return update()
