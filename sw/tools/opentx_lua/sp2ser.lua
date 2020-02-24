-- SmartPort to serial converter
-- TELEMETRY SCRIPT
--
-- Requires opentx compilation flags -DLUA=YES -DUSB_SERIAL=YES
--
-- On TX, set USB mode to serial
-- Add this script as telemetry script to your model

local function init_func()
end

local function bg_func()
  local sensorId, frameId, dataId, value = sportTelemetryPop()
  if frameId then
    local value0 = math.floor(value / 16777216) % 256
    local value1 = math.floor(value / 65536) % 256
    local value2 = math.floor(value / 256) % 256
    local value3 = value % 256
    serialWrite(string.char(value0) .. string.char(value1) .. string.char(value2) .. string.char(value3))
  end
end

local function run_func(event)
  lcd.drawText(1, 1, "sp2ser is running")
end

return {run = run_func, background = bg_func, init = init_func}
