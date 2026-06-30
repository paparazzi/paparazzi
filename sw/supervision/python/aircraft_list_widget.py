from typing import Dict, List

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QStyle,
    QStyleOptionButton,
    QStyleOptionFocusRect,
    QStyledItemDelegate,
    QStyleOptionViewItem,
    QToolTip,
    QVBoxLayout,
    QWidget,
)

from conf import Aircraft


AIRCRAFT_ROLE = QtCore.Qt.UserRole
ERROR_ROLE = QtCore.Qt.UserRole + 1
BUILD_STATUS_ROLE = QtCore.Qt.UserRole + 2
FLASH_STATUS_ROLE = QtCore.Qt.UserRole + 3
STATUS_BUTTON_WIDTH = 28
BUILD_STATUS_WIDTH = 24
FLASH_STATUS_WIDTH = 24
COLOR_SWATCH_WIDTH = 18
COLOR_SWATCH_SIZE = 12


class AircraftItemDelegate(QStyledItemDelegate):
    def check_rect(self, rect: QtCore.QRect, widget: QWidget) -> QtCore.QRect:
        check_option = QStyleOptionButton()
        check_rect = widget.style().subElementRect(QStyle.SE_CheckBoxIndicator, check_option, widget)
        check_rect.moveLeft(rect.left() + 6)
        check_rect.moveTop(rect.top() + ((rect.height() - check_rect.height()) // 2))
        return check_rect

    def color_rect(self, rect: QtCore.QRect, check_rect: QtCore.QRect) -> QtCore.QRect:
        return QtCore.QRect(
            check_rect.right() + 6 + ((COLOR_SWATCH_WIDTH - COLOR_SWATCH_SIZE) // 2),
            rect.top() + ((rect.height() - COLOR_SWATCH_SIZE) // 2),
            COLOR_SWATCH_SIZE,
            COLOR_SWATCH_SIZE,
        )

    def config_status_rect(self, rect: QtCore.QRect) -> QtCore.QRect:
        button_size = 22
        return QtCore.QRect(
            rect.right() - STATUS_BUTTON_WIDTH + ((STATUS_BUTTON_WIDTH - button_size) // 2),
            rect.top() + ((rect.height() - button_size) // 2),
            button_size,
            button_size,
        )

    def build_status_rect(self, rect: QtCore.QRect) -> QtCore.QRect:
        led_size = 13
        return QtCore.QRect(
            rect.right() - STATUS_BUTTON_WIDTH - FLASH_STATUS_WIDTH - BUILD_STATUS_WIDTH
            + ((BUILD_STATUS_WIDTH - led_size) // 2),
            rect.top() + ((rect.height() - led_size) // 2),
            led_size,
            led_size,
        )

    def flash_status_rect(self, rect: QtCore.QRect) -> QtCore.QRect:
        led_size = 13
        return QtCore.QRect(
            rect.right() - STATUS_BUTTON_WIDTH - FLASH_STATUS_WIDTH + ((FLASH_STATUS_WIDTH - led_size) // 2),
            rect.top() + ((rect.height() - led_size) // 2),
            led_size,
            led_size,
        )

    def draw_led(self, painter: QtGui.QPainter, rect: QtCore.QRect, status: str):
        status_colors = {
            "": QtGui.QColor("#ffffff"),
            "running": QtGui.QColor("#1f7ae0"),
            "success": QtGui.QColor("#2faa4f"),
            "error": QtGui.QColor("#d94343"),
        }
        painter.save()
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setBrush(status_colors.get(status, QtGui.QColor("#808080")))
        painter.setPen(QtGui.QPen(QtGui.QColor("#4d4d4d")))
        painter.drawEllipse(rect)
        painter.restore()

    def draw_aircraft_color(self, painter: QtGui.QPainter, rect: QtCore.QRect, ac: Aircraft):
        color = QtGui.QColor(ac.get_color())
        if not color.isValid():
            color = QtGui.QColor("#ffffff")
        painter.save()
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setBrush(color)
        painter.setPen(QtGui.QPen(QtGui.QColor("#4d4d4d")))
        painter.drawRoundedRect(rect, 2, 2)
        painter.restore()

    def paint(self, painter: QtGui.QPainter, option: QStyleOptionViewItem, index: QtCore.QModelIndex):
        opt = QStyleOptionViewItem(option)
        self.initStyleOption(opt, index)
        style = option.widget.style()

        selected = bool(opt.state & QStyle.State_Selected)
        if selected:
            background = opt.palette.highlight().color()
            foreground = opt.palette.highlightedText().color()
        else:
            background = QtGui.QColor("#dfeaf5" if index.row() % 2 else "#ffffff")
            foreground = opt.palette.text().color()
        painter.fillRect(option.rect, background)

        check_option = QStyleOptionButton()
        check_option.state = QStyle.State_Enabled
        if index.data(QtCore.Qt.CheckStateRole) == QtCore.Qt.Checked:
            check_option.state |= QStyle.State_On
        else:
            check_option.state |= QStyle.State_Off
        check_rect = self.check_rect(option.rect, option.widget)
        check_option.rect = check_rect
        style.drawControl(QStyle.CE_CheckBox, check_option, painter, option.widget)

        ac = index.data(AIRCRAFT_ROLE)
        color_rect = self.color_rect(option.rect, check_rect)
        if ac is not None:
            self.draw_aircraft_color(painter, color_rect, ac)

        text_rect = QtCore.QRect(
            check_rect.right() + 6 + COLOR_SWATCH_WIDTH + 4,
            option.rect.top(),
            max(
                0,
                option.rect.width() - check_rect.width() - COLOR_SWATCH_WIDTH - STATUS_BUTTON_WIDTH
                - BUILD_STATUS_WIDTH - FLASH_STATUS_WIDTH - 18,
            ),
            option.rect.height(),
        )
        painter.save()
        painter.setPen(foreground)
        painter.setFont(opt.font)
        text = opt.fontMetrics.elidedText(opt.text, QtCore.Qt.ElideRight, text_rect.width())
        painter.drawText(text_rect, QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, text)
        painter.restore()

        self.draw_led(painter, self.build_status_rect(option.rect), index.data(BUILD_STATUS_ROLE) or "")
        self.draw_led(painter, self.flash_status_rect(option.rect), index.data(FLASH_STATUS_ROLE) or "")

        error_msg = index.data(ERROR_ROLE) or ""
        icon_type = QStyle.SP_MessageBoxCritical if error_msg else QStyle.SP_DialogApplyButton
        status_rect = self.config_status_rect(option.rect)
        icon = style.standardIcon(icon_type)
        icon.paint(painter, status_rect, QtCore.Qt.AlignCenter)

        if opt.state & QStyle.State_HasFocus:
            focus_option = QStyleOptionFocusRect()
            focus_option.rect = option.rect.adjusted(1, 1, -1, -1)
            focus_option.state = opt.state
            focus_option.backgroundColor = background
            style.drawPrimitive(QStyle.PE_FrameFocusRect, focus_option, painter, option.widget)

    def sizeHint(self, option: QStyleOptionViewItem, index: QtCore.QModelIndex) -> QtCore.QSize:
        size = super().sizeHint(option, index)
        size.setHeight(max(size.height(), 26))
        return size


class AircraftListWidget(QWidget):
    selection_changed = QtCore.pyqtSignal()
    template_requested = QtCore.pyqtSignal(Aircraft)
    current_aircraft_changed = QtCore.pyqtSignal(Aircraft)
    color_requested = QtCore.pyqtSignal(Aircraft)
    menu_requested = QtCore.pyqtSignal(object, QtCore.QPoint)
    row_requested = QtCore.pyqtSignal(QListWidgetItem)

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.aircrafts: List[Aircraft] = []
        self.items: Dict[str, QListWidgetItem] = {}
        layout = QVBoxLayout(self)

        header = QWidget(self)
        header.setObjectName("aircraft_list_header")
        header.setStyleSheet("QWidget#aircraft_list_header { background: #e6e6e6; border-bottom: 1px solid #b8b8b8; }")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(6, 2, 4, 2)
        self.global_checkbox = QCheckBox(header)
        self.global_checkbox.setTristate(True)
        self.global_checkbox.setToolTip("Select all aircraft")
        self.global_checkbox.stateChanged.connect(self.handle_global_check_changed)
        header_layout.addWidget(self.global_checkbox)

        self.aircraft_header_label = QLabel("Aircraft", header)
        self.aircraft_header_label.setStyleSheet("font-weight: bold;")
        header_layout.addWidget(self.aircraft_header_label, 1)

        build_header = self._make_header_icon("Build", ":/icons/icons/build.png")
        header_layout.addWidget(build_header)
        flash_header = self._make_header_icon("Flash", ":/icons/icons/flash.png")
        header_layout.addWidget(flash_header)

        conf_header = QLabel(header)
        conf_header.setAlignment(QtCore.Qt.AlignCenter)
        conf_header.setToolTip("Status")
        conf_header.setFixedWidth(STATUS_BUTTON_WIDTH)
        conf_icon = self.style().standardIcon(QStyle.SP_DialogApplyButton)
        conf_header.setPixmap(conf_icon.pixmap(16, 16))
        header_layout.addWidget(conf_header)
        layout.addWidget(header)

        self.list_widget = QListWidget(self)
        self.delegate = AircraftItemDelegate(self.list_widget)
        self.list_widget.setItemDelegate(self.delegate)
        self.list_widget.setMouseTracking(True)
        self.list_widget.setUniformItemSizes(False)
        self.list_widget.itemChanged.connect(self.handle_item_changed)
        self.list_widget.currentItemChanged.connect(self.handle_current_item_changed)
        self.list_widget.itemDoubleClicked.connect(self.handle_item_double_clicked)
        self.list_widget.viewport().installEventFilter(self)
        layout.addWidget(self.list_widget)

    @staticmethod
    def _make_header_icon(tooltip: str, icon_path: str) -> QLabel:
        label = QLabel()
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setToolTip(tooltip)
        label.setFixedWidth(BUILD_STATUS_WIDTH if tooltip == "Build" else FLASH_STATUS_WIDTH)
        label.setPixmap(QtGui.QPixmap(icon_path).scaled(16, 16, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        return label

    def set_aircrafts(self, aircrafts: List[Aircraft]):
        self.aircrafts = aircrafts
        self.items.clear()
        with QtCore.QSignalBlocker(self.list_widget):
            self.list_widget.clear()
            for ac in aircrafts:
                item = QListWidgetItem("{} ({})".format(ac.name, ac.ac_id))
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                item.setCheckState(QtCore.Qt.Unchecked)
                item.setData(AIRCRAFT_ROLE, ac)
                item.setData(ERROR_ROLE, "")
                item.setData(BUILD_STATUS_ROLE, "")
                item.setData(FLASH_STATUS_ROLE, "")
                self.items[ac.name] = item
                self.list_widget.addItem(item)
        self.update_global_checkbox()
        if aircrafts:
            self.list_widget.setCurrentItem(self.items[aircrafts[0].name])

    def set_all(self, checked: bool):
        state = QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked
        with QtCore.QSignalBlocker(self.list_widget):
            for item in self.items.values():
                item.setCheckState(state)
        self.update_global_checkbox()
        self.selection_changed.emit()

    def handle_global_check_changed(self, state):
        if state == QtCore.Qt.PartiallyChecked:
            state = QtCore.Qt.Checked
        self.set_all(state == QtCore.Qt.Checked)

    def handle_item_changed(self, item: QListWidgetItem):
        self.update_global_checkbox()
        self.selection_changed.emit()

    def update_global_checkbox(self):
        states = [item.checkState() for item in self.items.values()]
        if not states or all(state == QtCore.Qt.Unchecked for state in states):
            state = QtCore.Qt.Unchecked
        elif all(state == QtCore.Qt.Checked for state in states):
            state = QtCore.Qt.Checked
        else:
            state = QtCore.Qt.PartiallyChecked
        with QtCore.QSignalBlocker(self.global_checkbox):
            self.global_checkbox.setCheckState(state)

    def checked_aircrafts(self) -> List[Aircraft]:
        return [ac for ac in self.aircrafts if self.items[ac.name].checkState() == QtCore.Qt.Checked]

    def set_error(self, ac: Aircraft, msg: str):
        self._set_item_data(ac, ERROR_ROLE, msg)

    def clear_error(self, ac: Aircraft):
        self._set_item_data(ac, ERROR_ROLE, "")

    def clear_errors(self):
        for ac in self.aircrafts:
            self.clear_error(ac)

    def set_build_status(self, ac: Aircraft, status: str):
        self._set_item_data(ac, BUILD_STATUS_ROLE, status)

    def clear_build_status(self, ac: Aircraft):
        self.set_build_status(ac, "")

    def clear_build_statuses(self, aircrafts: List[Aircraft]):
        for ac in aircrafts:
            self.clear_build_status(ac)

    def set_flash_status(self, ac: Aircraft, status: str):
        self._set_item_data(ac, FLASH_STATUS_ROLE, status)

    def clear_flash_status(self, ac: Aircraft):
        self.set_flash_status(ac, "")

    def _set_item_data(self, ac: Aircraft, role: int, value: str):
        item = self.items[ac.name]
        with QtCore.QSignalBlocker(self.list_widget):
            item.setData(role, value)
        self.list_widget.viewport().update(self.list_widget.visualItemRect(item))

    def handle_item_double_clicked(self, item: QListWidgetItem):
        ac = item.data(AIRCRAFT_ROLE)
        if ac is not None:
            self.template_requested.emit(ac)

    def handle_current_item_changed(self, current: QListWidgetItem, previous: QListWidgetItem):
        if current is not None:
            ac = current.data(AIRCRAFT_ROLE)
            if ac is not None:
                self.current_aircraft_changed.emit(ac)

    def eventFilter(self, watched, event):
        if watched == self.list_widget.viewport() and event.type() == QtCore.QEvent.ContextMenu:
            item = self.list_widget.itemAt(event.pos())
            if item is not None:
                ac = item.data(AIRCRAFT_ROLE)
                if ac is not None:
                    self.menu_requested.emit(ac, event.globalPos())
                    return True
            self.menu_requested.emit(None, event.globalPos())
            return True
        if watched == self.list_widget.viewport() and event.type() == QtCore.QEvent.MouseButtonRelease:
            item = self.list_widget.itemAt(event.pos())
            if item is not None and self.is_color_click(item, event.pos()):
                ac = item.data(AIRCRAFT_ROLE)
                if ac is not None:
                    self.color_requested.emit(ac)
                    return True
            if item is not None and not self.is_checkbox_click(item, event.pos()):
                self.row_requested.emit(item)
        if watched == self.list_widget.viewport() and event.type() == QtCore.QEvent.ToolTip:
            item = self.list_widget.itemAt(event.pos())
            if item is not None:
                tooltip = self.status_tooltip(item, event.pos())
                if tooltip:
                    QToolTip.showText(event.globalPos(), tooltip, self.list_widget.viewport())
                    return True
            QToolTip.hideText()
            return True
        return super().eventFilter(watched, event)

    def is_color_click(self, item: QListWidgetItem, pos: QtCore.QPoint) -> bool:
        rect = self.list_widget.visualItemRect(item)
        check_rect = self.delegate.check_rect(rect, self.list_widget)
        return self.delegate.color_rect(rect, check_rect).contains(pos)

    def is_checkbox_click(self, item: QListWidgetItem, pos: QtCore.QPoint) -> bool:
        rect = self.list_widget.visualItemRect(item)
        return self.delegate.check_rect(rect, self.list_widget).contains(pos)

    def status_tooltip(self, item: QListWidgetItem, pos: QtCore.QPoint) -> str:
        rect = self.list_widget.visualItemRect(item)
        if self.delegate.build_status_rect(rect).contains(pos):
            return self.led_tooltip("Build", item.data(BUILD_STATUS_ROLE) or "")
        if self.delegate.flash_status_rect(rect).contains(pos):
            return self.led_tooltip("Flash", item.data(FLASH_STATUS_ROLE) or "")
        if self.delegate.config_status_rect(rect).contains(pos):
            return item.data(ERROR_ROLE) or "Configuration OK"
        return ""

    @staticmethod
    def led_tooltip(name: str, status: str) -> str:
        labels = {
            "": "not run",
            "running": "running",
            "success": "success",
            "error": "error",
        }
        return "{}: {}".format(name, labels.get(status, status))
