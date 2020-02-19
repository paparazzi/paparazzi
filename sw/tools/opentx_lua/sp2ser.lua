-- SmartPort to serial converter
-- TELEMETRY SCRIPT

local function init_func()
end

local function bg_func()
end

local function run_func(event)
  lcd.drawText(1, 1, "Hello OpenTX!", DBLSIZE)
end

return {run = run_func, background = bg_func, init = init_func}