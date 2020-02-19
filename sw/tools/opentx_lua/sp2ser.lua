-- SmartPort to serial converter
-- TELEMETRY SCRIPT

local counter = -1

local function init_func()
end

local function bg_func()
--  local sensorId, frameId, dataId, value = sportTelemetryPop()
--  if frameID == 0x10 then -- DATA_FRAME
--    if dataId == 0x5015 then -- DATAID_DOWNLINK
--      counter = value
--    end
--  end
end

local function run_func(event)
  local sensorId, frameId, dataId, value = sportTelemetryPop()
  lcd.drawText(1, 1, "frameId: " .. frameId)
  lcd.drawText(1, 11, "dataId: " .. dataId)
  lcd.drawText(1, 21, "value: " .. value)
end

return {run = run_func, background = bg_func, init = init_func}