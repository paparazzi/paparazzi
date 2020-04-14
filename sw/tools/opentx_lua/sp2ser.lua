-- SmartPort to serial bridge
-- TELEMETRY SCRIPT
--
-- Requires opentx compilation flags -DLUA=YES -DUSB_SERIAL=YES -DCLI=NO
--
-- On TX, set hardware USB mode to serial
-- Add this script as telemetry script to your model

local downlink_str = ""
local uplink_str = ""

local function init_func()
end

local function downlink_bg()
  local sensorId, frameId, dataId, value = sportTelemetryPop()
  while frameId do
    if dataId == 0x5015 then
      local value0 = math.floor(value / 16777216) % 256
      local value1 = math.floor(value / 65536) % 256
      local value2 = math.floor(value / 256) % 256
      local value3 = value % 256
      serialWrite(string.char(value0) .. string.char(value1) .. string.char(value2) .. string.char(value3))
      downlink_str = string.char(value0) .. string.char(value1) .. string.char(value2) .. string.char(value3)
    end
    sensorId, frameId, dataId, value = sportTelemetryPop()
  end
end

local uplink_buffer = ""
local function uplink_bg()
-- Read bytes from serial
  local free = 4 - string.len(uplink_buffer)
  local in_str = serialRead(free)
  if string.len(in_str) > 0 then
    uplink_buffer = uplink_buffer .. in_str
  end
  -- Uplink if uplink buffer full
  if string.len(uplink_buffer) == 4 then
    local sensorId = 0
    local frameId = 0
    local dataId = 0x5016
    local value = string.byte(uplink_buffer, 1) * 16777216 + string.byte(uplink_buffer, 2) * 65536 + string.byte(uplink_buffer, 3) * 256 + string.byte(uplink_buffer, 4)
    sportTelemetryPush(sensorId, frameId, dataId, value)
    uplink_str = uplink_buffer
    uplink_buffer = ""
  end
end

local function bg_func()
  downlink_bg()
  uplink_bg()
end

local function run_func(event)
  lcd.clear()
  lcd.drawText(1, 1, "sp2ser is running...")
  lcd.drawText(1, 11, "Downlink: '" .. downlink_str .. "'")
  lcd.drawText(1, 21, "Uplink: '" .. uplink_str .. "'")
end

return {run = run_func, background = bg_func, init = init_func}
