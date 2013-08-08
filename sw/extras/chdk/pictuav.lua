--[[
@title PictUAV
@param d Display off frame 0=never
@default d 0
--]]

-- Developed on IXUS 230 HS. Other cameras should work as well, but some errors or crashes are possible
-- also considering that some firmwares still have CHDK bugs.
--
-- Other settings in the camera that you should try to manipulate to reduce the time between taking pictures:
-- CHDK menu
--   Extra Photo overrides
--     Disable overrides : Off
--   Do not override shutter speed  ( I allow the camera to determine exposure time, but feel free to experiment)
--   ND filter state: Out (removes ND filter to allow more light to fall on the sensor )
--   Override subj. dist. value: 65535. (good to keep it there)
--   Do not save RAW images, they take longer
--   
-- Camera menu
--   Use P mode, certainly not automatic
--   If you can hardset the focus, do this towards infinity
--   Play around with aperture. Smaller apertures are better for more sharpness, but not every camera allows you to.
--      (Some cameras have really bad behaviour on larger apertures that allow more light through, especially at edges).
--   Disable IS  (plane vibrations mess up its function and increases the chance of blur)
--   Take Large images with Fine resolution.
--   Turn "Review" mode off.
--   Consider hard-setting ISO, but consider local weather. If set too high, shutter time goes up, which causes blur. 
--     Blur can then also occur in areas where there is little light reflection from the earth.
--   
--   How to use the script:
--     Load this on the card under "CHDK/SCRIPTS"
--     Enter the CHDK menu through your "ALT" button
--     Under scripts, select the script and specify "Autostart: on"
--
--   As the camera starts up, this also loads. with the shutter button pressed, you can interrupt the script
--   Then press the "ALT" button to disable the scripting actuator. 
--   Press the shutter button to extend the lens
--   Press "ALT" again to bring up the scripting actuator.
--   Press the shutter button to reinitiate the script.
--   If you have a IXUS 230HS like me, the focus can't be set automatically. Point the camera at a distant object while the script 
--   is starting. It should say "Focused", after which it's ready for use.
--
--   Example paparazzi airframe configuration:
--
--    <section name="DIGITAL_CAMERA" prefix="DC_">
--      <define name="AUTOSHOOT_QUARTERSEC_PERIOD" value="8" unit="quarter_second"/>
--      <define name="AUTOSHOOT_METER_GRID" value="60" unit="meter"/>
--      <define name="SHUTTER_DELAY" value="0" unit="quarter_second"/>
--      <define name="POWER_OFF_DELAY" value="3" unit="quarter_second"/>
--    </section>
--
--    <load name="digital_cam.xml" >
--      <define name="DC_SHUTTER_LED" value="4"/>
--      <define name="DC_POWER_OFF_LED" value="4"/>
--      <define name="DC_RELEASE" value="LED_ON" />  
--      <define name="DC_PUSH" value="LED_OFF" />
--    </load>
--

print( "PictUAV Started " )

function print_status (frame)
   local free = get_jpg_count()
   print("#" .. frame )
end

-- switch to autofocus mode, pre-focus, then go to manual focus mode (locking focus there).
-- this helps to reduce the delay between the signal and taking the picture.
function pre_focus()
   local focused = false
   local try = 1
   while not focused and try <= 5 do
      print("Pre-focus attempt " .. try)
      press("shoot_half")
      sleep(2000)
      if get_prop(18) > 0 then
	 print("Focused")
	 focused = true
	 set_aflock(1)
      end
      release("shoot_half")
      sleep(500)
      try = try + 1
   end
   return focused
end

-- set aperture/shooting mode to landscape
set_prop(6,3)

ap = get_prop(6)
print ("AF(3=inf,4=MF) "..ap)

-- Turn IS off
set_prop(145, 3)

-- set P mode
set_capture_mode(2)
sleep(1000)

-- Get focusing mode
p = get_prop(6)
if (p==3) then
	print "Inf set."
else
	-- on ixus230hs, no explicit MF. 
	-- so set to infinity (3)
	while (p ~= 3) do
		press "left"
		release "left"
		p = get_prop (6)
	end
	print "Focus set to infinity"
end

-- on ixus230hs set focus doesn't fail, but doesn't do anything.
set_focus(100)
print "set_focus 100"
sleep(2000)
f = get_focus

-- on ixus230hs set focus doesn't fail, but doesn't do anything.
set_focus( 65535 )
print "set_focus 65535"
sleep( 2000)
g = get_focus

if (f==g) then 
	print "set_focus inop"
	-- if focusing until here didn't work, pre-focus using a different method.
	pre_focus()
else
	-- set_aflock(1) fails when the camera isn't knowingly focused.
	print( "Setting aflock 1" )
	sleep(1000)
	set_aflock( 1 )
end

-- measuring the pulse on CHDK isn't necessarily that accurate, they can easily vary by 40ms.
-- Since I'm using a 100ms wait here, the variance for a shoot is around 140ms. 
-- If the pulse is 250ms, then I should allow for anything down to 110.
-- For Paparazzi, taking a single picture using the button may not work because the timing may be very off
-- considering the 4Hz loop (this requires a change in the cam module for paparazzi).

print( "PictUAV loop " )
a = -1
repeat
	a = get_usb_power(0)
	-- a pulse is detected longer than ~610ms.
	if (a>61) then
	        print( "shutting down " )
		shut_down()
		sleep(1500)
		break
	end
	-- a pulse longer than ~100ms is detected.
	if (a>10) then

	   frame = 1
	   shoot()

	   frame = frame + 1

	end
	sleep(100)
until ( false )
print( "PictUAV ended " )

