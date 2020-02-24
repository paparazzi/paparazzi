-- SmartPort to serial converter
-- TELEMETRY SCRIPT
--
-- Requires -DLUA=YES -DUSB_SERIAL=YES

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
  lcd.drawText(1, 1, "sp2ser")
  if frameId then
		lcd.drawText(1, 11, "frameId: " .. frameId)
		lcd.drawText(1, 21, "dataId: " .. dataId)
		lcd.drawText(1, 31, "value: " .. value)

		local value0 = math.floor(value / 16777216) % 256
		local value1 = math.floor(value / 65536) % 256
		local value2 = math.floor(value / 256) % 256
		local value3 = value % 256
		serialWrite(string.char(value0) .. string.char(value1) .. string.char(value2) .. string.char(value3))
	end
end

return {run = run_func, background = bg_func, init = init_func}
