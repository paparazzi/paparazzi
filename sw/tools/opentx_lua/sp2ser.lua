-- SmartPort to serial bridge
-- TELEMETRY SCRIPT
--
-- Requires opentx compilation flags -DLUA=YES -DUSB_SERIAL=YES -DCLI=NO
--
-- On TX, set hardware USB mode to serial
-- Add this script as telemetry script to your model

local in_str = ""
local disp_str = "Waiting for serial input... "

local function init_func()
end

local function bg_func()
  local sensorId, frameId, dataId, value = sportTelemetryPop()
  while frameId do
    if dataId == 0x5015 then
      local value0 = math.floor(value / 16777216) % 256
      local value1 = math.floor(value / 65536) % 256
      local value2 = math.floor(value / 256) % 256
      local value3 = value % 256
      serialWrite(string.char(value0) .. string.char(value1) .. string.char(value2) .. string.char(value3))
    end
    sensorId, frameId, dataId, value = sportTelemetryPop()
  end
  
  in_str = serialRead()
  if in_str ~= "" then
    if string.len(disp_str) > 0 and 
        (string.sub(in_str, -1) == "\n" or string.sub(in_str, -1) == "\r") then
      disp_str = ""
    else
      disp_str = disp_str .. in_str
    end
  end
end

local function run_func(event)
  lcd.clear()
  lcd.drawText(1, 1, "sp2ser is running...")
  lcd.drawText(1, 11, disp_str)
  if string.len(in_str) > 0 then
    lcd.drawText(1, 21, "(" .. string.byte(in_str) .. ") '" .. in_str .. "'")
  end
end

return {run = run_func, background = bg_func, init = init_func}
