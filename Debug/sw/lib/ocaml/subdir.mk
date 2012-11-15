################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/lib/ocaml/base64.o \
../sw/lib/ocaml/convert.o \
../sw/lib/ocaml/cserial.o \
../sw/lib/ocaml/debug.o \
../sw/lib/ocaml/defivybus.o \
../sw/lib/ocaml/editAirframe.o \
../sw/lib/ocaml/egm96.o \
../sw/lib/ocaml/env.o \
../sw/lib/ocaml/expr_lexer.o \
../sw/lib/ocaml/expr_parser.o \
../sw/lib/ocaml/expr_syntax.o \
../sw/lib/ocaml/extXml.o \
../sw/lib/ocaml/fig.o \
../sw/lib/ocaml/geometry_2d.o \
../sw/lib/ocaml/gm.o \
../sw/lib/ocaml/gtk_papget_editor.o \
../sw/lib/ocaml/gtk_papget_gauge_editor.o \
../sw/lib/ocaml/gtk_papget_led_editor.o \
../sw/lib/ocaml/gtk_papget_text_editor.o \
../sw/lib/ocaml/gtk_tools.o \
../sw/lib/ocaml/http.o \
../sw/lib/ocaml/iGN.o \
../sw/lib/ocaml/latlong.o \
../sw/lib/ocaml/logpprz.o \
../sw/lib/ocaml/mapCanvas.o \
../sw/lib/ocaml/mapFP.o \
../sw/lib/ocaml/mapGoogle.o \
../sw/lib/ocaml/mapIGN.o \
../sw/lib/ocaml/mapTrack.o \
../sw/lib/ocaml/mapWaypoints.o \
../sw/lib/ocaml/maps_support.o \
../sw/lib/ocaml/ml_gtk_drag.o \
../sw/lib/ocaml/ocaml_tools.o \
../sw/lib/ocaml/os_calls.o \
../sw/lib/ocaml/papget.o \
../sw/lib/ocaml/papget_common.o \
../sw/lib/ocaml/papget_renderer.o \
../sw/lib/ocaml/platform.o \
../sw/lib/ocaml/pprz.o \
../sw/lib/ocaml/serial.o \
../sw/lib/ocaml/srtm.o \
../sw/lib/ocaml/ubx.o \
../sw/lib/ocaml/wind_sock.o \
../sw/lib/ocaml/xbee.o \
../sw/lib/ocaml/xml2h.o \
../sw/lib/ocaml/xmlCom.o \
../sw/lib/ocaml/xmlEdit.o 

C_SRCS += \
../sw/lib/ocaml/caml_from_c_example.c \
../sw/lib/ocaml/convert.c \
../sw/lib/ocaml/cserial.c \
../sw/lib/ocaml/ml_gtk_drag.c \
../sw/lib/ocaml/ml_gtkgl_hack.c 

OBJS += \
./sw/lib/ocaml/caml_from_c_example.o \
./sw/lib/ocaml/convert.o \
./sw/lib/ocaml/cserial.o \
./sw/lib/ocaml/ml_gtk_drag.o \
./sw/lib/ocaml/ml_gtkgl_hack.o 

C_DEPS += \
./sw/lib/ocaml/caml_from_c_example.d \
./sw/lib/ocaml/convert.d \
./sw/lib/ocaml/cserial.d \
./sw/lib/ocaml/ml_gtk_drag.d \
./sw/lib/ocaml/ml_gtkgl_hack.d 


# Each subdirectory must supply rules for building sources it contributes
sw/lib/ocaml/%.o: ../sw/lib/ocaml/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


