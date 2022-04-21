# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
all:
	mkdir -p generated
	pyuic5 ui/conf_header.ui -o generated/ui_conf_header.py
	pyuic5 ui/conf_item.ui -o generated/ui_conf_item.py
	pyuic5 ui/build.ui -o generated/ui_build.py
	pyuic5 ui/configuration_panel.ui -o generated/ui_configuration_panel.py
	pyuic5 ui/new_ac_dialog.ui -o generated/ui_new_ac_dialog.py
	pyuic5 ui/session.ui -o generated/ui_session.py
	pyuic5 ui/program.ui -o generated/ui_program.py
	pyuic5 ui/operation_panel.ui -o generated/ui_operation_panel.py
	pyuic5 ui/console.ui -o generated/ui_console.py
	pyuic5 ui/tools_list.ui -o generated/ui_tools_list.py
	pyuic5 ui/app_settings.ui -o generated/ui_app_settings.py

clean_ui:
	rm -r generated
