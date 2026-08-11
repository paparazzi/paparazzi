/**
 * @file messages.cpp
 * @brief Paparazzi Telemetry Messages Viewer.
 *
 * This file implements the Qt-based UI for monitoring and inspecting Ivy telemetry messages.
 * It dynamically parses the unit and coefficient geometries to reflect real-time telemetry
 * variables.
 */

#include <QAbstractItemView>
#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QDialog>
#include <QDrag>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QGuiApplication>
#include <QHash>
#include <QHBoxLayout>
#include <QIcon>
#include <QInputDialog>
#include <QIODevice>
#include <QLabel>
#include <QList>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMainWindow>
#include <QMimeData>
#include <QMouseEvent>
#include <QPoint>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollBar>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QString>
#include <QStringList>
#include <QStyle>
#include <QTabWidget>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector>
#include <QWidget>
#include <QXmlStreamReader>

#include "../../include/os_desktop_utils.h"
#include "../../include/pprz_version.h"
#include "pprzlinkQt/IvyQtLink.h"

/**
 * @brief Represents the MessagesConfig struct.
 * @details This struct encapsulates the primary logic and UI structures required
 * for MessagesConfig operations, ensuring robust and memory-safe management within the telemetry
 * pipeline.
 */
struct MessagesConfig
{
    QString ivyBus;
    QStringList classes;
    bool timestamp {false};
    bool force {false};
    QString geometry;
    MessagesConfig() = default;
};

/**
 * @brief Per-field rendering metadata, cached on a message's first arrival.
 * @details Mirrors what pprzlink's messages.ml resolves once per field: the
 * scaling coefficient and alternate unit used to render "raw (scaled alt_unit)",
 * plus the enum value table used to render "NAME (index)".
 */
struct FieldDisplay
{
    double coef = 1.0; // alt_unit_coef (1.0 = no scaling)
    QString altUnit; // non-empty -> append " (coef*raw <altUnit>)"
    QStringList enumValues; // non-empty -> integer shown as "NAME (index)"
    QString format; // 'format' attribute (e.g. "%.1f"); empty -> default
};

/**
 * @brief Represents the MsgTracker struct.
 * @details This struct encapsulates the primary logic and UI structures required
 * for MsgTracker operations, ensuring robust and memory-safe management within the telemetry
 * pipeline.
 */
struct MsgTracker
{
    QLabel *timeLabel = nullptr;
    QWidget *timeBox = nullptr;
    QVector<QLabel *> fieldLabels;
    QVector<class DraggableButton *> fieldButtons;
    // Per-field rendering metadata (coef, alt_unit, enum table), resolved once
    // when the message's page is first built.
    QVector<FieldDisplay> fieldDisplays;
    qint64 lastUpdateMs = 0;
    bool isGreen = false;
    int lastSecs = -1;
};

/**
 * @brief Represents the SenderTab class.
 * @details This class encapsulates the primary logic and UI structures required
 * for SenderTab operations, ensuring robust and memory-safe management within the telemetry
 * pipeline.
 */
class SenderTab : public QWidget
{
    Q_OBJECT
public:
    explicit SenderTab(const QString &senderName,
                       const QString &className,
                       pprzlink::MessageDictionary *dict,
                       QWidget *parent = nullptr);
    void handleMessage(const pprzlink::Message &msg);

private:
    void updateTimers();

    QString m_senderName;
    QString m_className;
    pprzlink::MessageDictionary *m_dict;

    QListWidget *m_listWidget;
    QStackedWidget *m_stackedWidget;

    int m_listWidgetWidth = 100;
    void updateListWidgetWidth(int contentWidth);

    QHash<QString, MsgTracker> m_msgTrackers;
};

/**
 * @brief Represents the MessagesWindow class.
 * @details This class encapsulates the primary logic and UI structures required
 * for MessagesWindow operations, ensuring robust and memory-safe management within the telemetry
 * pipeline.
 */
class MessagesWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MessagesWindow(const MessagesConfig &config, QWidget *parent = nullptr);
    ~MessagesWindow() override;

private:
    MessagesConfig m_config;
    QTabWidget *m_classTabWidget;
    QLabel *m_waitingLabel;
    QHash<QString, SenderTab *> m_senderTabs;
    pprzlink::MessageDictionary *m_dict = nullptr;
    pprzlink::IvyQtLink *m_link = nullptr;

    void setupDictionaryAndLink();
};

/**
 * @brief Represents the FieldInfo struct.
 * @details This struct encapsulates the primary logic and UI structures required
 * for FieldInfo operations, ensuring robust and memory-safe management within the telemetry
 * pipeline.
 */
struct FieldInfo
{
    QString coef; // alt_unit_coef (explicit, or resolved via units.xml)
    QString baseUnit; // 'unit' attribute -> shown in the field button label
    QString altUnit; // 'alt_unit' attribute -> appended to the scaled value
    QStringList values; // enum 'values' (split on '|'); empty when not an enum
    QString format; // 'format' attribute (printf, e.g. "%.1f"); empty if none
};
static QHash<QString, QHash<QString, QHash<QString, FieldInfo>>> s_fieldInfos;
static constexpr int GREEN_DECAY_RATE_MS = 200;

#include <algorithm>

/**
 * @brief One row of the pprzlink unit-conversion table (units.xml).
 * @details Mirrors a <unit from="..." to="..." coef="..." [auto="code|display"]/>
 * entry. The optional @c automode reproduces pprzlink's airframe/display
 * auto-conversion semantics so the lookup matches the canonical ground segment.
 */
struct UnitConv
{
    QString from;
    QString to;
    QString automode; // "code", "display" or empty
    double coef = 1.0;
};
static QVector<UnitConv> s_unitConvs;

/**
 * @brief Loads the default unit-conversion table (units.xml).
 * @param unitsXmlPath Absolute path to units.xml (lives next to messages.xml).
 * @details This table is what lets a field declared as e.g. unit="mm"
 * alt_unit="m" be displayed in meters even though messages.xml carries no
 * explicit alt_unit_coef. Missing/malformed entries are skipped rather than
 * aborting, so a partial table still scales every unit it does know about.
 */
static void loadUnitsTable(const QString &unitsXmlPath)
{
    s_unitConvs.clear();
    QFile file(unitsXmlPath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Could not open" << unitsXmlPath << "for unit conversions";
        return;
    }
    QXmlStreamReader xml(&file);
    while (!xml.atEnd() && !xml.hasError()) {
        if (xml.readNext() == QXmlStreamReader::StartElement
            && xml.name() == QLatin1String("unit")) {
            const auto attrs = xml.attributes();
            UnitConv u;
            u.from = attrs.value(QLatin1String("from")).toString();
            u.to = attrs.value(QLatin1String("to")).toString();
            u.automode = attrs.value(QLatin1String("auto")).toString().toLower();
            bool ok = false;
            u.coef = attrs.value(QLatin1String("coef")).toString().toDouble(&ok);
            if (ok && u.coef != 0.0 && !u.from.isEmpty() && !u.to.isEmpty()) {
                s_unitConvs.append(u);
            }
        }
    }
    if (xml.hasError()) {
        qWarning() << "XML error in" << unitsXmlPath << ":" << xml.errorString();
    }
}

/**
 * @brief Conversion factor turning a value in @p fromUnit into @p toUnit.
 * @return The matching coefficient, or 1.0 when no conversion is known.
 * @details Faithful re-implementation of pprzlink's OCaml @c scale_of_units
 * (without the airframe "auto" override): identical units need no scaling;
 * otherwise the first units.xml row whose from/to match wins, honouring the
 * permissive auto="display"/auto="code" rows exactly as the reference does. An
 * unknown pair falls back to 1.0 so a value is never silently zeroed.
 */
static double scaleOfUnits(const QString &fromUnit, const QString &toUnit)
{
    if (fromUnit == toUnit) {
        return 1.0;
    }
    for (const UnitConv &u : std::as_const(s_unitConvs)) {
        // pprzlink resolves message scales with ~auto:"display": rows that carry
        // an auto="code"/"display" attribute are matched on their target unit
        // alone; every other row needs an exact from/to match. First hit wins.
        const bool match
                = u.automode.isEmpty() ? (u.from == fromUnit && u.to == toUnit) : (u.to == toUnit);
        if (match) {
            return u.coef;
        }
    }
    return 1.0;
}

/**
 * @brief Render a double exactly like OCaml's @c Stdlib.string_of_float.
 * @param x The value to format.
 * @return The OCaml-style textual representation.
 * @details messages.ml prints scalar floats through @c PprzLink.string_of_value,
 * which for a float calls @c string_of_float: the number is formatted with
 * @c "%.12g" and, when the result reads as a bare integer (only digits and an
 * optional leading sign), a trailing @c '.' is appended so the lexeme is
 * unmistakably a float. Hence 12.0 -> "12." and 185.0 -> "185.", while 12.5,
 * 0.001 and 1e-07 are already float-looking and pass through untouched.
 * @c QString::asprintf formats in the C locale, matching OCaml's decimal point.
 */
static QString ocamlStringOfFloat(double x)
{
    const QString s = QString::asprintf("%.12g", x);
    // OCaml's valid_float_lexem: append '.' only when every character is a digit
    // or a '-' (i.e. there is no '.', exponent 'e', or inf/nan letter already).
    bool bare = !s.isEmpty();
    for (const QChar c : s) {
        if (!(c.isDigit() || c == QLatin1Char('-'))) {
            bare = false;
            break;
        }
    }
    return bare ? s + QLatin1Char('.') : s;
}

/**
 * @brief Apply a messages.xml @c format to a floating value, OCaml-style.
 * @param fmt The printf format string taken from the field's @c format attribute.
 * @param value The value to format.
 * @return The formatted text, or a null QString when @p fmt is not a single safe
 *         floating-point conversion (the caller then falls back to
 *         string_of_float, mirroring OCaml where an incompatible format raises).
 * @details messages.ml renders a field that carries a @c format through
 * @c PprzLink.formatted_string_of_value, which for a float is
 * @c sprintf (Scanf.format_from_string format "%f") -- i.e. the format must be
 * type-compatible with @c "%f". Hence @c "%.1f" 12.0 -> "12.0". Non-printf
 * markers such as @c "csv" raise in OCaml and are rejected here so we degrade to
 * the default rendering instead. The accepted grammar is a single conversion
 * @c %[flags][width][.precision]<float-spec> with no @c %% , @c * , positional
 * @c $ , length modifier or @c %n, which also defuses any format-string attack
 * even though messages.xml is a trusted generated file.
 */
static QString applyFloatFormat(const QString &fmt, double value)
{
    static const QRegularExpression re(
            QStringLiteral("\\A%[-+ 0#]*[0-9]*(?:\\.[0-9]+)?[eEfFgGaA]\\z"));
    if (!re.match(fmt).hasMatch()) {
        return {}; // signal: fall back to string_of_float
    }
    return QString::asprintf(fmt.toUtf8().constData(), value);
}

static void loadUnitCoefs(const QString &xmlPath)
{
    // The unit-conversion table sits next to messages.xml (both are generated
    // into $PAPARAZZI_HOME/var by the build). Load it first so fields that rely
    // on an implicit conversion (unit + alt_unit but no alt_unit_coef) scale.
    loadUnitsTable(QFileInfo(xmlPath).absolutePath() + QStringLiteral("/units.xml"));

    QFile file(xmlPath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Could not open" << xmlPath << "to parse unit coefs";
        return;
    }
    QXmlStreamReader xml(&file);
    QString currentClass;
    QString currentMessage;
    while (!xml.atEnd() && !xml.hasError()) {
        QXmlStreamReader::TokenType token = xml.readNext();
        if (token == QXmlStreamReader::StartElement) {
            auto name = xml.name();
            if (name == QLatin1String("msg_class")) {
                currentClass = xml.attributes().value(QLatin1String("name")).toString();
            } else if (name == QLatin1String("message")) {
                currentMessage = xml.attributes().value(QLatin1String("name")).toString();
            } else if (name == QLatin1String("field")) {
                auto attrs = xml.attributes();
                QString fieldName = attrs.value(QLatin1String("name")).toString();
                // Resolve the display metadata exactly like pprzlink's messages.ml:
                //   * the field button shows the BASE unit -> "<type> <name> (<unit>): ";
                //   * the value shows "raw (alt_unit_coef*raw  alt_unit)" whenever an
                //     alt_unit exists, the coefficient coming from an explicit
                //     alt_unit_coef or, failing that, the units.xml table (mm->m=0.001);
                //   * enum fields (a "values" list) are shown as "NAME (index)".
                const QString rawUnit = attrs.value(QLatin1String("unit")).toString();
                const QString altUnit = attrs.value(QLatin1String("alt_unit")).toString();
                const QString explicitCoef = attrs.value(QLatin1String("alt_unit_coef")).toString();
                const QString valuesAttr = attrs.value(QLatin1String("values")).toString();
                const QString formatAttr = attrs.value(QLatin1String("format")).toString();

                FieldInfo info;
                info.coef = explicitCoef.isEmpty()
                        ? QString::number(scaleOfUnits(rawUnit, altUnit), 'g', 12)
                        : explicitCoef;
                info.baseUnit = rawUnit;
                info.altUnit = altUnit;
                info.format = formatAttr;
                if (!valuesAttr.isEmpty()) {
                    info.values = valuesAttr.split(QLatin1Char('|'), Qt::SkipEmptyParts);
                }

                s_fieldInfos[currentClass][currentMessage][fieldName] = info;
            }
        } else if (token == QXmlStreamReader::EndElement) {
            auto name = xml.name();
            if (name == QLatin1String("msg_class"))
                currentClass.clear();
            else if (name == QLatin1String("message"))
                currentMessage.clear();
        }
    }
    if (xml.hasError()) {
        qWarning() << "XML error in" << xmlPath << ":" << xml.errorString();
    }
}

static QString senderIdToString(const std::variant<QString, uint8_t> &senderV)
{
    if (std::holds_alternative<QString>(senderV)) {
        return std::get<QString>(senderV).trimmed();
    }
    return QString::number(static_cast<int>(std::get<uint8_t>(senderV)));
}

static QString safeFieldName(const QString &fieldName, int index)
{
    if (!fieldName.isEmpty()) {
        return fieldName;
    }
    return QStringLiteral("field_%1").arg(index);
}

static QString safeMessageName(const QString &msgName)
{
    if (!msgName.isEmpty()) {
        return msgName;
    }
    return QStringLiteral("<unknown message>");
}

/**
 * @brief Represents the DraggableButton class.
 * @details This class encapsulates the primary logic and UI structures required
 * for DraggableButton operations, ensuring robust and memory-safe management within the telemetry
 * pipeline.
 */
class DraggableButton : public QPushButton
{
public:
    DraggableButton(const QString &text,
                    const QString &senderName,
                    const QString &className,
                    const QString &msgName,
                    const QString &fieldName,
                    const QString &coef,
                    bool isArray,
                    int arraySize,
                    QWidget *parent = nullptr)
        : QPushButton(text, parent)
        , m_senderName(senderName)
        , m_className(className)
        , m_msgName(msgName)
        , m_fieldName(fieldName)
        , m_coef(coef)
        , m_isArray(isArray)
        , m_arraySize(arraySize)
    {
        setToolTip(QStringLiteral("Drag-and-drop field on:\n\t- Real-Time Plotter to plot a "
                                  "curve\n\t- GCS map to display as a papget"));
    }

    void setDynamicArraySize(int sz)
    {
        if (m_isArray)
            m_arraySize = sz;
    }

protected:
    void mousePressEvent(QMouseEvent *event) override
    {
        QPushButton::mousePressEvent(event);
        if (event->button() == Qt::LeftButton) {
            m_dragStartPos = event->pos();
        }
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (!(event->buttons() & Qt::LeftButton))
            return;
        if ((event->pos() - m_dragStartPos).manhattanLength() < QApplication::startDragDistance())
            return;

        QDrag *drag = new QDrag(this);
        QMimeData *mimeData = new QMimeData;
        QString delayedFilePath;
        if (m_isArray) {
            delayedFilePath = QStringLiteral("/tmp/pprz_dnd_%1_%2.txt")
                                      .arg(QCoreApplication::applicationPid())
                                      .arg(QDateTime::currentMSecsSinceEpoch());
            QFile::remove(delayedFilePath); // ensure clear
            mimeData->setText(QStringLiteral("delayed_array:") + delayedFilePath);
        } else {
            QString payload = m_senderName + QStringLiteral(":") + m_className + QStringLiteral(":")
                    + m_msgName + QStringLiteral(":") + m_fieldName + QStringLiteral(":") + m_coef;
            mimeData->setText(payload);
        }
        drag->setMimeData(mimeData);

        Qt::DropAction action = drag->exec(Qt::CopyAction | Qt::MoveAction);
        drag->deleteLater(); // Prevent QDrag memory leak on repeated drops

        if (m_isArray && action != Qt::IgnoreAction) {
            QString defaultRange = (m_arraySize > 0) ? QStringLiteral("0-%1").arg(m_arraySize - 1)
                                                     : QStringLiteral("0");

            QInputDialog dialog(nullptr);
            dialog.setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
            dialog.setWindowTitle(QStringLiteral("Index of value to plot"));
            dialog.setLabelText(QStringLiteral("Index or range in the array?"));
            dialog.setTextValue(defaultRange);

            QStringList parts;
            if (dialog.exec() == QDialog::Accepted) {
                QString text = dialog.textValue();
                QList<int> indices;
                QStringList tokens = text.split(QRegularExpression(QStringLiteral("[,;\\s]+")),
                                                Qt::SkipEmptyParts);
                for (const QString &token : std::as_const(tokens)) {
                    if (token.contains(QLatin1String("-"))) {
                        QStringList range = token.split(QStringLiteral("-"));
                        if (range.size() == 2) {
                            int start = range[0].toInt();
                            int end = range[1].toInt();
                            if (start <= end) {
                                for (int i = start; i <= end; ++i) {
                                    if (i >= 0 && (m_arraySize == 0 || i < m_arraySize))
                                        indices.append(i);
                                }
                            }
                        }
                    } else {
                        int i = token.toInt();
                        if (i >= 0 && (m_arraySize == 0 || i < m_arraySize))
                            indices.append(i);
                    }
                }
                for (int n : indices) {
                    parts << QStringLiteral("%1:%2:%3:%4[%5]:%6")
                                     .arg(m_senderName)
                                     .arg(m_className)
                                     .arg(m_msgName)
                                     .arg(m_fieldName)
                                     .arg(n)
                                     .arg(m_coef);
                }

                QFile f(delayedFilePath);
                if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    QTextStream out(&f);
                    out << parts.join('\n');
                    f.close();
                }
            } else {
                // write empty to unblock receiver
                QFile f(delayedFilePath);
                if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    QTextStream out(&f);
                    out << "";
                    f.close();
                }
            }
        }
    }

private:
    QPoint m_dragStartPos;
    QString m_senderName;
    QString m_className;
    QString m_msgName;
    QString m_fieldName;
    QString m_coef;
    bool m_isArray;
    int m_arraySize;
};

SenderTab::SenderTab(const QString &senderName,
                     const QString &className,
                     pprzlink::MessageDictionary *dict,
                     QWidget *parent)
    : QWidget(parent)
    , m_senderName(senderName)
    , m_className(className)
    , m_dict(dict)
{
    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    m_listWidget = new QListWidget(this);
    m_listWidget->setFrameShape(QFrame::NoFrame);
    m_listWidget->setAttribute(Qt::WA_MacShowFocusRect, false);
    m_listWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    m_listWidget->setMinimumWidth(100);
    // m_listWidget->setMaximumWidth(100);
    m_listWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    m_listWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    m_stackedWidget = new QStackedWidget(this);
    m_stackedWidget->setMinimumWidth(100);
    m_stackedWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    layout->addWidget(m_listWidget);
    layout->addWidget(m_stackedWidget, 1);

    connect(m_listWidget,
            &QListWidget::currentRowChanged,
            m_stackedWidget,
            &QStackedWidget::setCurrentIndex);

    QTimer *globalTimer = new QTimer(this);
    connect(globalTimer, &QTimer::timeout, this, &SenderTab::updateTimers);
    globalTimer->start(50);
}

void SenderTab::updateTimers()
{
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    for (auto &t : m_msgTrackers) {
        if (!t.timeLabel || !t.timeBox || t.lastUpdateMs == 0) {
            continue;
        }

        qint64 msecs = now - t.lastUpdateMs;
        msecs = std::max<qint64>(msecs, 0);

        if (t.isGreen && msecs > GREEN_DECAY_RATE_MS) {
            t.timeBox->setStyleSheet(
                    QStringLiteral(".QWidget { background-color: #000000; border-radius: 0px; "
                                   "}\nQLabel { color: #fff; font-weight: bold; }"));
            t.isGreen = false;
            t.lastSecs = -1; // force text update
        }

        if (msecs > 1999) {
            int secs = static_cast<int>(msecs / 1000);
            if (secs != t.lastSecs) {
                t.timeLabel->setText(QString::number(secs));
                t.lastSecs = secs;
            }
        } else if (t.lastSecs != 0) {
            t.timeLabel->setText(QLatin1String(""));
            t.lastSecs = 0;
        }
    }
}

void SenderTab::updateListWidgetWidth(int contentWidth)
{
    const int scrollbarWidth = m_listWidget->verticalScrollBar()->isVisible()
            ? m_listWidget->verticalScrollBar()->sizeHint().width()
            : m_listWidget->style()->pixelMetric(QStyle::PM_ScrollBarExtent);
    const int extraPadding = 20; // extra buffer for item margins and layout spacing
    const int desiredWidth = qMax(100, contentWidth + scrollbarWidth + extraPadding);

    if (desiredWidth <= m_listWidgetWidth)
        return;

    m_listWidgetWidth = desiredWidth;
    m_listWidget->setMinimumWidth(m_listWidgetWidth);
    m_listWidget->setMaximumWidth(m_listWidgetWidth);
}

void SenderTab::handleMessage(const pprzlink::Message &msg)
{
    QString msgName = safeMessageName(msg.getDefinition().getName());

    if (!m_msgTrackers.contains(msgName)) {
        QWidget *page = new QWidget(this);
        QVBoxLayout *vlayout = new QVBoxLayout(page);

        // Item in list widget
        QListWidgetItem *item = new QListWidgetItem();

        // Custom widget for list item
        QWidget *itemWidget = new QWidget(m_listWidget);
        itemWidget->setStyleSheet(QStringLiteral("background: transparent;"));
        QHBoxLayout *itemLayout = new QHBoxLayout(itemWidget);
        itemLayout->setContentsMargins(4, 2, 4, 2);

        QLabel *nameLabel = new QLabel(msgName);
        nameLabel->setAlignment(Qt::AlignCenter);

        QLabel *timeLabel = new QLabel(QLatin1String(""));
        timeLabel->setMinimumWidth(40);
        timeLabel->setAlignment(Qt::AlignCenter);

        QWidget *timeBox = new QWidget();
        QHBoxLayout *tBoxL = new QHBoxLayout(timeBox);
        tBoxL->setContentsMargins(2, 2, 2, 2);
        tBoxL->addWidget(timeLabel);
        timeBox->setStyleSheet(
                QStringLiteral(".QWidget { background-color: #000000; border-radius: 0px; "
                               "}\nQLabel { color: #fff; font-weight: bold; }"));

        itemLayout->addStretch();
        itemLayout->addWidget(nameLabel);
        itemLayout->addStretch();
        itemLayout->addWidget(timeBox);

        itemWidget->adjustSize();
        item->setSizeHint(itemWidget->sizeHint());
        item->setData(Qt::UserRole, msgName);
        updateListWidgetWidth(itemWidget->sizeHint().width());

        int insertRow = 0;
        for (; insertRow < m_listWidget->count(); ++insertRow) {
            auto *widgetItem = m_listWidget->item(insertRow);
            if (widgetItem) {
                const QString existingName = widgetItem->data(Qt::UserRole).toString();
                if (existingName > msgName)
                    break;
            }
        }
        m_listWidget->insertItem(insertRow, item);
        m_listWidget->setItemWidget(item, itemWidget);
        m_stackedWidget->insertWidget(insertRow, page);

        MsgTracker tracker;
        tracker.timeLabel = timeLabel;
        tracker.timeBox = timeBox;

        // Fields for the right page
        const auto &def = msg.getDefinition();
        const int nbFields = static_cast<int>(def.getNbFields());
        tracker.fieldLabels.resize(nbFields);
        tracker.fieldButtons.resize(nbFields);
        tracker.fieldDisplays.resize(nbFields);
        for (int i = 0; i < nbFields; ++i) {
            const auto &field = def.getField(i);
            QHBoxLayout *hlayout = new QHBoxLayout();

            QString fieldName = safeFieldName(field.getName(), i);
            QString typeName = field.getType().toString();

            QString coef = QStringLiteral("1.");
            QString baseUnit;
            QString altUnit;
            QStringList enumValues;
            QString fieldFormat;
            auto classIt = s_fieldInfos.constFind(m_className);
            if (classIt != s_fieldInfos.constEnd()) {
                auto msgIt = classIt->constFind(msgName);
                if (msgIt != classIt->constEnd()) {
                    auto fieldIt = msgIt->constFind(fieldName);
                    if (fieldIt != msgIt->constEnd()) {
                        coef = fieldIt->coef;
                        baseUnit = fieldIt->baseUnit;
                        altUnit = fieldIt->altUnit;
                        enumValues = fieldIt->values;
                        fieldFormat = fieldIt->format;
                    }
                }
            }

            // Button label mirrors messages.ml's `sprintf "%s %s %s: "`: the type,
            // the field name and the BASE unit in parentheses (e.g.
            // "int32 alt (mm): "). The parenthesis is dropped when there is no unit.
            const QString unitPart = baseUnit.isEmpty()
                    ? QString()
                    : QStringLiteral("(") + baseUnit + QStringLiteral(")");
            QString btnText = typeName + QStringLiteral(" ") + fieldName + QStringLiteral(" ")
                    + unitPart + QStringLiteral(": ");

            // Parse the textual coefficient once, here, so the hot value-update
            // path only multiplies doubles. A missing/invalid or zero coef is
            // treated as 1.0 (no scaling) so a malformed entry can never blank
            // out or zero a field.
            bool coefOk = false;
            const double coefValue = coef.toDouble(&coefOk);
            FieldDisplay &fd = tracker.fieldDisplays[i];
            fd.coef = (coefOk && coefValue != 0.0) ? coefValue : 1.0;
            fd.altUnit = altUnit;
            fd.enumValues = enumValues;
            fd.format = fieldFormat;

            DraggableButton *btn = new DraggableButton(btnText,
                                                       m_senderName,
                                                       m_className,
                                                       msgName,
                                                       fieldName,
                                                       coef,
                                                       field.getType().isArray(),
                                                       field.getType().getArraySize(),
                                                       page);
            QLabel *valLabel = new QLabel(QStringLiteral("XXXX"), page);

            hlayout->addWidget(btn);
            hlayout->addWidget(valLabel);
            hlayout->addStretch();

            vlayout->addLayout(hlayout);
            tracker.fieldLabels[i] = valLabel;
            tracker.fieldButtons[i] = btn;
        }
        vlayout->addStretch();
        m_msgTrackers.insert(msgName, tracker);
    }

    // Update values
    auto it = m_msgTrackers.find(msgName);
    if (it == m_msgTrackers.end()) {
        qWarning() << "Received message with unknown name" << msgName;
        return;
    }

    MsgTracker &tracker = it.value();
    if (!tracker.timeLabel || !tracker.timeBox) {
        qWarning() << "Invalid tracker for message" << msgName;
        return;
    }

    tracker.lastUpdateMs = QDateTime::currentMSecsSinceEpoch();
    if (tracker.lastSecs != 0) {
        tracker.timeLabel->setText(QLatin1String(""));
        tracker.lastSecs = 0;
    }

    // Briefly flash green background
    if (!tracker.isGreen) {
        tracker.timeBox->setStyleSheet(
                QStringLiteral(".QWidget { background-color: #22ff22; border-radius: 0px; "
                               "}\nQLabel { color: #000; font-weight: bold; }"));
        tracker.isGreen = true;
    }

    const auto &def = msg.getDefinition();
    const int nbFields = static_cast<int>(def.getNbFields());
    for (int i = 0; i < nbFields; ++i) {
        if (i >= tracker.fieldLabels.size() || !tracker.fieldLabels[i]) {
            continue;
        }

        try {
            auto rv = msg.getRawValue(i);
            const auto &type = def.getField(i).getType();

            // Per-field rendering metadata resolved when the page was first built.
            static const FieldDisplay s_emptyFd;
            const FieldDisplay &fd
                    = (i < tracker.fieldDisplays.size()) ? tracker.fieldDisplays[i] : s_emptyFd;
            const double coef = fd.coef;
            const bool hasAlt = !fd.altUnit.isEmpty();

            if (!type.isArray()) {
                // Reproduce messages.ml's scalar rendering:
                //   * an enum field (a "values" list) shows "NAME (index)";
                //   * a field carrying a printf "format" is rendered through it
                //     (formatted_string_of_value), e.g. "%.1f" 12.0 -> "12.0";
                //   * otherwise integers use string_of_int and floats use
                //     string_of_float (so a whole value keeps its dot: 12.0 ->
                //     "12.").
                // When an alt_unit exists the scaled part " (coef*v  alt_unit)"
                // (%f, 6 decimals) is appended, where v is parsed back from the
                // displayed raw string exactly as OCaml's alt_value does
                // (float_of_string value) -- so any format truncation carries
                // through, e.g. "%.2f" 0.123456 -> "0.12 (6.876000deg)".
                auto render = [&](double raw, bool isIntegral) -> QString {
                    if (isIntegral && !fd.enumValues.isEmpty()) {
                        const int idx = static_cast<int>(raw);
                        if (idx >= 0 && idx < fd.enumValues.size()) {
                            return fd.enumValues.at(idx) + QStringLiteral(" (")
                                    + QString::number(idx) + QStringLiteral(")");
                        }
                    }
                    QString rawStr;
                    if (!isIntegral && !fd.format.isEmpty()) {
                        rawStr = applyFloatFormat(fd.format, raw); // null if unusable
                    }
                    if (rawStr.isNull()) {
                        rawStr = isIntegral ? QString::number(static_cast<qlonglong>(raw))
                                            : ocamlStringOfFloat(raw);
                    }
                    if (!hasAlt) {
                        return rawStr;
                    }
                    bool baseOk = false;
                    const double base = rawStr.toDouble(&baseOk);
                    return rawStr + QStringLiteral(" (")
                            + QString::number(coef * (baseOk ? base : raw), 'f', 6) + fd.altUnit
                            + QStringLiteral(")");
                };
                switch (type.getBaseType()) {
                case pprzlink::BaseType::CHAR: {
                    char v;
                    rv.getValue(v);
                    tracker.fieldLabels[i]->setText(render(static_cast<double>(v), true));
                } break;
                case pprzlink::BaseType::INT8: {
                    int8_t v;
                    rv.getValue(v);
                    tracker.fieldLabels[i]->setText(render(static_cast<double>(v), true));
                } break;
                case pprzlink::BaseType::INT16: {
                    int16_t v;
                    rv.getValue(v);
                    tracker.fieldLabels[i]->setText(render(static_cast<double>(v), true));
                } break;
                case pprzlink::BaseType::INT32: {
                    int32_t v;
                    rv.getValue(v);
                    tracker.fieldLabels[i]->setText(render(static_cast<double>(v), true));
                } break;
                case pprzlink::BaseType::UINT8: {
                    uint8_t v;
                    rv.getValue(v);
                    tracker.fieldLabels[i]->setText(render(static_cast<double>(v), true));
                } break;
                case pprzlink::BaseType::UINT16: {
                    uint16_t v;
                    rv.getValue(v);
                    tracker.fieldLabels[i]->setText(render(static_cast<double>(v), true));
                } break;
                case pprzlink::BaseType::UINT32: {
                    uint32_t v;
                    rv.getValue(v);
                    tracker.fieldLabels[i]->setText(render(static_cast<double>(v), true));
                } break;
                case pprzlink::BaseType::FLOAT: {
                    float v;
                    rv.getValue(v);
                    tracker.fieldLabels[i]->setText(render(static_cast<double>(v), false));
                } break;
                case pprzlink::BaseType::DOUBLE: {
                    double v;
                    rv.getValue(v);
                    tracker.fieldLabels[i]->setText(render(v, false));
                } break;
                case pprzlink::BaseType::STRING: {
                    QString v;
                    rv.getValue(v);
                    tracker.fieldLabels[i]->setText(v);
                } break;
                default: {
                    rv.setOutputInt8AsInt(true);
                    std::stringstream ss;
                    ss << rv;
                    tracker.fieldLabels[i]->setText(QString::fromStdString(ss.str()));
                } break;
                }
            } else {
                // Arrays are shown raw (unscaled), matching messages.ml which
                // does not scale comma-separated values. Float/double arrays are
                // rendered element-by-element with the field's "format" when it is
                // a usable printf conversion, else string_of_float (so 12.0 ->
                // "12."), joined by ',' like OCaml; other element types keep the
                // pprzlink stream formatting. The element count is still probed to
                // keep the drag-to-plot index dialog in sync with dynamic arrays.
                int dynSize = 0;
                QString floatArrayText;
                bool haveFloatArray = false;
                auto fmtElem = [&](double e) -> QString {
                    QString s;
                    if (!fd.format.isEmpty()) {
                        s = applyFloatFormat(fd.format, e);
                    }
                    return s.isNull() ? ocamlStringOfFloat(e) : s;
                };
                try {
                    switch (type.getBaseType()) {
                    case pprzlink::BaseType::CHAR: {
                        std::vector<char> v;
                        rv.getValue(v);
                        dynSize = v.size();
                    } break;
                    case pprzlink::BaseType::INT8: {
                        std::vector<int8_t> v;
                        rv.getValue(v);
                        dynSize = v.size();
                    } break;
                    case pprzlink::BaseType::INT16: {
                        std::vector<int16_t> v;
                        rv.getValue(v);
                        dynSize = v.size();
                    } break;
                    case pprzlink::BaseType::INT32: {
                        std::vector<int32_t> v;
                        rv.getValue(v);
                        dynSize = v.size();
                    } break;
                    case pprzlink::BaseType::UINT8: {
                        std::vector<uint8_t> v;
                        rv.getValue(v);
                        dynSize = v.size();
                    } break;
                    case pprzlink::BaseType::UINT16: {
                        std::vector<uint16_t> v;
                        rv.getValue(v);
                        dynSize = v.size();
                    } break;
                    case pprzlink::BaseType::UINT32: {
                        std::vector<uint32_t> v;
                        rv.getValue(v);
                        dynSize = v.size();
                    } break;
                    case pprzlink::BaseType::FLOAT: {
                        std::vector<float> v;
                        rv.getValue(v);
                        dynSize = v.size();
                        QStringList elems;
                        for (float e : v) {
                            elems << fmtElem(static_cast<double>(e));
                        }
                        floatArrayText = elems.join(QLatin1Char(','));
                        haveFloatArray = true;
                    } break;
                    case pprzlink::BaseType::DOUBLE: {
                        std::vector<double> v;
                        rv.getValue(v);
                        dynSize = v.size();
                        QStringList elems;
                        for (double e : v) {
                            elems << fmtElem(e);
                        }
                        floatArrayText = elems.join(QLatin1Char(','));
                        haveFloatArray = true;
                    } break;
                    default:
                        break;
                    }
                    if (dynSize > 0 && i < tracker.fieldButtons.size() && tracker.fieldButtons[i]) {
                        tracker.fieldButtons[i]->setDynamicArraySize(dynSize);
                    }
                } catch (...) {
                    // Best-effort element-count probe; on failure keep going so the
                    // raw field text is still rendered below.
                    qDebug() << "Array size probe failed for" << msgName << "field index" << i;
                }
                if (haveFloatArray) {
                    tracker.fieldLabels[i]->setText(floatArrayText);
                } else {
                    rv.setOutputInt8AsInt(true);
                    std::stringstream ss;
                    ss << rv;
                    tracker.fieldLabels[i]->setText(QString::fromStdString(ss.str()));
                }
            }
        } catch (const std::exception &ex) {
            qWarning() << "Failed to read field value for" << msgName << "field index" << i << ":"
                       << ex.what();
        } catch (...) {
            qWarning() << "Unknown error while updating field value for" << msgName;
        }
    }
}

MessagesWindow::MessagesWindow(const MessagesConfig &config, QWidget *parent)
    : QMainWindow(parent)
    , m_config(config)
{
    setWindowTitle(QStringLiteral("Messages"));

    if (!m_config.geometry.isEmpty()) {
        QRegularExpression re(QStringLiteral(R"(^(\d+)x(\d+)(?:\+[-]?(\d+)\+[-]?(\d+))?$)"));
        QRegularExpressionMatch match = re.match(m_config.geometry);
        if (match.hasMatch()) {
            int w = match.captured(1).toInt();
            int h = match.captured(2).toInt();
            resize(w, h);
            if (!match.captured(3).isEmpty() && !match.captured(4).isEmpty()) {
                int x = match.captured(3).toInt();
                int y = match.captured(4).toInt();
                move(x, y);
            }
        } else {
            resize(600, 400);
        }
    } else {
        resize(600, 400);
    }

    QWidget *cntral = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(cntral);
    layout->setContentsMargins(0, 0, 0, 0);

    m_waitingLabel = new QLabel(QStringLiteral("Initializing telemetry..."), this);
    m_waitingLabel->setAlignment(Qt::AlignCenter);

    m_classTabWidget = new QTabWidget(this);
    m_classTabWidget->hide();

    layout->addWidget(m_waitingLabel);
    layout->addWidget(m_classTabWidget);
    setCentralWidget(cntral);

    setupDictionaryAndLink();
}

MessagesWindow::~MessagesWindow()
{
    if (m_link) {
        m_link->stop();
        delete m_link;
    }
    delete m_dict;
}

void MessagesWindow::setupDictionaryAndLink()
{
    QString phome = qgetenv("PAPARAZZI_HOME");
    if (phome.isEmpty())
        phome = QStringLiteral("/home/%1/paparazzi").arg(qgetenv("USER"));
    QString xmlPath = phome + QStringLiteral("/var/messages.xml");

    if (!QFile::exists(xmlPath)) {
        m_waitingLabel->setText(tr("Missing messages.xml at %1").arg(xmlPath));
        qWarning() << "Missing messages.xml at" << xmlPath;
        return;
    }

    loadUnitCoefs(xmlPath);

    try {
        m_dict = new pprzlink::MessageDictionary(xmlPath);
        if (!m_dict) {
            throw std::runtime_error("Failed to allocate MessageDictionary");
        }
        m_link = new pprzlink::IvyQtLink(*m_dict, QStringLiteral("messages_qt"), this);
        if (!m_link) {
            throw std::runtime_error("Failed to allocate IvyQtLink");
        }
        m_waitingLabel->setText(tr("Waiting for Ivy bus..."));
        connect(m_link, &pprzlink::IvyQtLink::serverConnected, this, [this]() {
            if (m_waitingLabel && m_waitingLabel->isVisible()) {
                m_waitingLabel->setText(tr("Connected to Ivy bus just fine,\nbut still waiting for "
                                           "telemetry data..."));
            }
        });
        m_link->start(m_config.ivyBus.isEmpty() ? QStringLiteral("127.255.255.255:2010")
                                                : m_config.ivyBus);
        if (m_waitingLabel && m_waitingLabel->isVisible()) {
            m_waitingLabel->setText(tr("Waiting for telemetry data..."));
        }
    } catch (const std::exception &ex) {
        qWarning() << "Failed to initialize messaging:" << ex.what();
        m_waitingLabel->setText(tr("Telemetry initialization failed"));
        delete m_link;
        m_link = nullptr;
        delete m_dict;
        m_dict = nullptr;
        return;
    } catch (...) {
        qWarning() << "Failed to initialize messaging due to unknown error.";
        m_waitingLabel->setText(tr("Telemetry initialization failed"));
        delete m_link;
        m_link = nullptr;
        delete m_dict;
        m_dict = nullptr;
        return;
    }

    if (!m_dict) {
        m_waitingLabel->setText(tr("Telemetry dictionary unavailable"));
        return;
    }

    struct ClassFilter
    {
        QString className;
        QString sender;
    };
    QList<ClassFilter> filters;
    QStringList classArgs = m_config.classes;
    if (classArgs.isEmpty()) {
        classArgs.append(QStringLiteral("telemetry:*"));
    }

    for (const QString &c : classArgs) {
        QStringList parts = c.split(':');
        ClassFilter f;
        f.className = parts[0];
        f.sender = (parts.size() > 1) ? parts[1] : QStringLiteral("*");
        filters.append(f);
    }

    for (const ClassFilter &filter : filters) {
        QString className = filter.className;
        const auto msgs = m_dict->getMsgsForClass(className);
        if (msgs.empty()) {
            qWarning() << "No message definitions found for class" << className;
            continue;
        }

        for (const auto &def : msgs) {
            if (!m_link)
                break;

            // To emulate OCaml's non-force behavior on the local side, we can filter out creating
            // tabs until ALIVE is seen, or we just bind everything as IvyQtLink doesn't support
            // sender-specific binding anyway.

            m_link->BindMessage(
                    def,
                    this,
                    [this, className, filter, def](const QString &sender,
                                                   const pprzlink::Message &msg) {
                        QString sId = sender.trimmed();
                        if (sId.isEmpty()) {
                            const auto &senderV = msg.getSenderId();
                            sId = senderIdToString(senderV);
                        }
                        if (sId.isEmpty()) {
                            sId = QStringLiteral("ground");
                        }

                        if (filter.sender != QLatin1String("*") && filter.sender != sId) {
                            return; // Skip messages from unrequested sender
                        }

                        if (!m_config.force && className == QLatin1String("telemetry")
                            && filter.sender == QLatin1String("*")
                            && def.getName() != QLatin1String("ALIVE")) {
                            // Only start tracking a sender if we've seen ALIVE, or if we have force
                            // enabled
                            QString tabName = className + QStringLiteral(":") + sId;
                            if (!m_senderTabs.contains(tabName)) {
                                return;
                            }
                        }

                        if (m_waitingLabel && m_waitingLabel->isVisible()) {
                            m_waitingLabel->hide();
                            if (m_classTabWidget) {
                                m_classTabWidget->show();
                            }
                        }

                        QString tabName = className + QStringLiteral(":") + sId;
                        if (!m_senderTabs.contains(tabName)) {
                            SenderTab *tab = new SenderTab(sId, className, m_dict, this);
                            m_classTabWidget->addTab(tab, tabName);
                            m_senderTabs[tabName] = tab;
                        }

                        m_senderTabs[tabName]->handleMessage(msg);
                    });
        }
    }
}

/**
 * @brief Application entry point.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status code.
 */
int main(int argc, char *argv[])
{
    // Set metadata BEFORE application instantiation to prevent XDG portal double-registration
    // root cause ("Connection already associated with an application ID").
    QCoreApplication::setApplicationVersion(QStringLiteral(PPRZ_VERSION_DESC));
    // Set internal names in lowercase with underscores for safe XDG folder paths
    // QCoreApplication::setOrganizationName("paparazzi"); // only for settings, not really relevant
    // here
    // Mint de XDG underscore for filesystem compatibility
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_messages"));
    QCoreApplication::setApplicationName(QStringLiteral("paparazzi-messages"));

    QApplication app(argc, argv);
    QApplication::setStyle(new EditorLighteningStyle(QApplication::style()));

    QCommandLineParser parser;
    parser.setSingleDashWordOptionMode(QCommandLineParser::ParseAsLongOptions);
    parser.setApplicationDescription(QStringLiteral("Paparazzi Messages Viewer"));
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption ivyBusOption(
            QStringLiteral("b"),
            QStringLiteral("Ivy bus (default 127.255.255.255:2010)"),
            QStringLiteral("bus"),
            qEnvironmentVariable("IVY_BUS", QStringLiteral("127.255.255.255:2010")));
    parser.addOption(ivyBusOption);
    QCommandLineOption classOption(
            QStringLiteral("c"),
            QStringLiteral(
                    "Class name to listen to (can be used multiple times, e.g. telemetry:*)"),
            QStringLiteral("class"));
    parser.addOption(classOption);
    QCommandLineOption timestampOption(
            QStringLiteral("timestamp"),
            QStringLiteral("Bind to timestamped messages (currently ignored)"));
    parser.addOption(timestampOption);
    QCommandLineOption forceOption(
            QStringLiteral("force"),
            QStringLiteral("Force waiting on all messages, not only ALIVE for telemetry class"));
    parser.addOption(forceOption);
    QCommandLineOption geometryOption(
            QStringLiteral("g"),
            QStringLiteral("Set the window geometry (e.g., '500x500+100+100')"),
            QStringLiteral("geometry"));
    parser.addOption(geometryOption);

    QStringList args = QApplication::arguments();
    QStringList mergedArgs;
    for (int i = 0; i < args.size(); ++i) {
        QString arg = args[i];
        if ((arg.startsWith('\'') && !arg.endsWith('\''))
            || (arg.startsWith('"') && !arg.endsWith('"'))) {
            QChar quoteType = arg[0];
            QString merged = arg;
            int j = i + 1;
            bool foundClosed = false;
            while (j < args.size()) {
                merged += QStringLiteral(" ") + args[j];
                if (args[j].endsWith(quoteType)) {
                    foundClosed = true;
                    break;
                }
                j++;
            }
            if (foundClosed) {
                i = j;
                mergedArgs.append(merged.mid(1, merged.length() - 2));
            } else {
                mergedArgs.append(arg);
            }
        } else if ((arg.startsWith('\'') && arg.endsWith('\'') && arg.length() >= 2)
                   || (arg.startsWith('"') && arg.endsWith('"') && arg.length() >= 2)) {
            mergedArgs.append(arg.mid(1, arg.length() - 2));
        } else {
            mergedArgs.append(arg);
        }
    }

    parser.process(mergedArgs);

    MessagesConfig config;
    config.ivyBus = parser.value(ivyBusOption);
    config.classes = parser.values(classOption);
    config.timestamp = parser.isSet(timestampOption);
    config.force = parser.isSet(forceOption);
    config.geometry = parser.value(geometryOption);

    QString iconPath = QStringLiteral(":/penguin_icon_msg.png");
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(
            QApplication::desktopFileName(),
            QStringLiteral("Paparazzi Messages"),
            QStringLiteral(
                    "View and inspect telemetry messages sent from or to Aircraft or elsewhere"),
            iconPath,
            QStringLiteral("paparazzi-messages"));

    QApplication::setWindowIcon(icon);

    MessagesWindow window(config);
    window.setWindowIcon(icon);
    window.show();

    return QApplication::exec();
}

#include "messages.moc"
