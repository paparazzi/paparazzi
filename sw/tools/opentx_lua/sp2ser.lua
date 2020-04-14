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
      downlink_str = string.format("0x %X %X %X %X", value0, value1, value2, value3)
    end
    sensorId, frameId, dataId, value = sportTelemetryPop()
  end
end

local function uplink_bg()
  local in_str = serialRead(4)
  local len = string.len(in_str)
  if len > 0 then
    local sensorId = 0
    local frameId = len
    local dataId = 0x5016
    local value = 0
    local byte = 16777216
    for i=1,len do
      value = value + byte * string.byte(in_str, i)
      byte = byte / 256
    end
    sportTelemetryPush(sensorId, frameId, dataId, value)
    uplink_str = string.format("0x%X (%s)", value, frameId)
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
