/**
 * @file gaia.cpp
 * @brief Paparazzi Gaia: World Environment Simulator UI.
 *
 * @details
 * This file provides the graphical user interface for the Gaia simulator component
 * within the Paparazzi UAV framework. It broadcasts environmental parameters
 * (e.g., wind speed, updrafts, sim time scaling, GPS availability) via the Ivy bus.
 * The telemetry values are encapsulated in the WORLD_ENV message schema.
 *
 * It uses the Qt6 framework for UI construction and the pprzlink library for
 * cross-process Ivy communication.
 */
#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDebug>
#include <QDial>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFrame>
#include <QGridLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QMainWindow>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QPoint>
#include <QPolygon>
#include <QSizePolicy>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "../include/os_desktop_utils.h"
#include "../include/pprz_version.h"
#include "pprzlinkQt/IvyQtLink.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr int SENDING_PERIOD_MS = 5000;

/**
 * @class ArrowDial
 * @brief Custom Qt dial widget rendered as a directional arrow compass.
 *
 * @details
 * Inherits from QDial to provide a 360-degree rotational selector. Instead of a
 * standard knob, it heavily overrides the paintEvent to manually draw an arrowhead
 * mapping strictly to wind direction angles.
 */
class ArrowDial : public QDial
{
    Q_OBJECT
public:
    explicit ArrowDial(QWidget *parent = nullptr)
        : QDial(parent)
    {
        setMinimumSize(100, 100);
        setWrapping(true);
        setNotchesVisible(true);
    }

protected:
    /**
     * @brief Custom render loop overriding the default QDial drawing.
     * @param event The triggered paint event.
     *
     * @details Uses QPainter to scale, translate, and orient a geometric polygon
     * (the arrow) dynamically matched to the current bounding angle (0 to 359).
     */
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);

        painter.translate(width() / 2.0, height() / 2.0);
        double side = qMin(width(), height());
        painter.scale(side / 200.0, side / 200.0);

        QColor edgeColor(255, 255, 255, 255); // White edge for contrast
        QColor fillColor(0, 0, 0, 255);

        QPen pen = painter.pen();
        pen.setWidth(1);
        pen.setColor(edgeColor);
        painter.setPen(pen);
        painter.setBrush(fillColor);

        painter.setRenderHint(QPainter::Antialiasing);

        painter.save();

        double minimum = this->minimum();
        double maximum = this->maximum();
        double val = this->value();
        double percent = (val - minimum) / (maximum - minimum);
        double angle = 360.0 * percent;

        painter.rotate(angle);

        static const QPolygon polygon(
                {QPoint(0, 80), QPoint(8, -80), QPoint(0, -75), QPoint(-8, -80)});

        painter.drawPolygon(polygon, Qt::OddEvenFill);
        painter.restore();

        if (notchesVisible()) {
            painter.save();
            for (int i = 0; i < 4; i++) {
                painter.drawRect(-2, 85, 4, 13);
                painter.rotate(90);
            }
            painter.restore();

            painter.save();
            painter.rotate(45);
            for (int i = 0; i < 4; i++) {
                painter.drawRect(-1, 90, 2, 8);
                painter.rotate(90);
            }
            painter.restore();

            painter.save();
            for (int i = 0; i < 360; i += 10) {
                if (i % 90 != 0) {
                    painter.drawRect(-1, 94, 2, 4);
                }
                painter.rotate(10);
            }
            painter.restore();
        }
    }
};

/**
 * @class GaiaWindow
 * @brief The main graphical window for the Gaia environment simulation.
 *
 * @details
 * Constructs the layout, spins up the UI elements (sliders, dials, and spinboxes),
 * and establishes a bi-directional Ivy network link. It leverages a single QTimer
 * cycle to continuously dump environmental telemetry onto the bus.
 */
class GaiaWindow : public QMainWindow
{
    Q_OBJECT

public:
    GaiaWindow(const QString &ivyBus,
               double timeScale,
               double windSpeed,
               double windDir,
               double windUp,
               bool gpsOff,
               QWidget *parent = nullptr);
    ~GaiaWindow() override;

private:
    /**
     * @brief Periodically broadcasts internal environmental states to the Ivy network.
     *
     * @details Trigonometrically breaks down the wind direction and magnitude into East/North
     * vectors. Binds all scalar values safely into the pprzlink::Message schema.
     */
    void sendWorldEnv();

    /**
     * @brief Instantiates and layouts all Qt widgets.
     * @param initTimeScale Initial time scaling factor.
     * @param initWindSpeed Initial wind magnitude.
     * @param initWindDir Initial wind direction (degrees).
     * @param initWindUp Initial vertical updraft magnitude.
     * @param initGpsOff Emulated GPS signal blocking toggle.
     *
     * @details Uses a localized lambda to neatly stack coupled layout blocks (QLabel + QSlider +
     * QDoubleSpinBox).
     */
    void setupUI(double initTimeScale,
                 double initWindSpeed,
                 double initWindDir,
                 double initWindUp,
                 bool initGpsOff);
    /**
     * @brief Initializes the Paparazzi Ivy datalink.
     * @param ivyBus The IP:Port binding address for Ivy broadcasts.
     *
     * @details Parses the global internal XML message dictionaries to understand the WORLD_ENV
     * schema before allowing safe bindings.
     */
    void setupIvy(const QString &ivyBus);

    QDoubleSpinBox *m_spinTimeScaleVal;
    ArrowDial *m_dialWindDir;
    QDoubleSpinBox *m_spinWindSpeedVal;
    QDoubleSpinBox *m_spinWindUpVal;
    QDoubleSpinBox *m_spinIrContrastVal;
    QCheckBox *m_chkGpsOff;
    QTimer *m_timer;

    std::unique_ptr<pprzlink::MessageDictionary> m_dict;
    std::unique_ptr<pprzlink::IvyQtLink> m_link;
    pprzlink::MessageDefinition m_worldEnvDef;
    bool m_defFound = false;
};

GaiaWindow::GaiaWindow(const QString &ivyBus,
                       double timeScale,
                       double windSpeed,
                       double windDir,
                       double windUp,
                       bool gpsOff,
                       QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("Gaia"));
    resize(500, 250);

    setupUI(timeScale, windSpeed, windDir, windUp, gpsOff);
    setupIvy(ivyBus);

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &GaiaWindow::sendWorldEnv);
    m_timer->start(SENDING_PERIOD_MS);
}

GaiaWindow::~GaiaWindow()
{
    if (m_timer) {
        m_timer->stop();
    }
    if (m_link) {
        m_link->stop();
        m_link.reset();
    }
    m_dict.reset();
}

void GaiaWindow::setupUI(double initTimeScale,
                         double initWindSpeed,
                         double initWindDir,
                         double initWindUp,
                         bool initGpsOff)
{
    QWidget *central = new QWidget(this);
    QHBoxLayout *mainLayout = new QHBoxLayout(central);

    // Left side: sliders
    QFrame *slidersFrame = new QFrame();
    slidersFrame->setFrameShape(QFrame::NoFrame);
    QVBoxLayout *slidersLayout = new QVBoxLayout(slidersFrame);
    slidersLayout->setSpacing(12);

    auto createSliderBlock = [this, slidersLayout](const QString &text,
                                                   double min,
                                                   double max,
                                                   double step,
                                                   int decimals,
                                                   double current,
                                                   bool isVisible = true) -> QDoubleSpinBox * {
        QWidget *container = new QWidget();
        QVBoxLayout *containerLayout = new QVBoxLayout(container);
        containerLayout->setContentsMargins(0, 0, 0, 0);
        QHBoxLayout *textLayout = new QHBoxLayout();
        QLabel *titleLabel = new QLabel(text);
        QDoubleSpinBox *spinBox = new QDoubleSpinBox();
        spinBox->setDecimals(decimals);
        spinBox->setRange(min, max);
        spinBox->setSingleStep(step);
        spinBox->setValue(current);
        // spinBox->setButtonSymbols(QAbstractSpinBox::NoButtons); //Whatever you fancy
        spinBox->setMinimumWidth(40);
        spinBox->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        // By default spinboxes show up/down arrows
        textLayout->addWidget(titleLabel);
        textLayout->addWidget(spinBox);

        QSlider *slider = new QSlider(Qt::Horizontal);
        int factor = pow(10, decimals);
        slider->setRange(min * factor, max * factor);
        slider->setSingleStep(step * factor);
        slider->setValue(current * factor);

        containerLayout->addLayout(textLayout);
        containerLayout->addWidget(slider);

        slidersLayout->addWidget(container);
        container->setVisible(isVisible);

        // Connect slider and spinbox
        connect(slider, &QSlider::valueChanged, this, [spinBox, factor](int value) {
            spinBox->setValue(value / static_cast<double>(factor));
        });
        connect(spinBox,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this,
                [slider, factor](double value) {
                    slider->setValue(static_cast<int>(std::round(value * factor)));
                });
        connect(spinBox,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this,
                [this](double) { this->sendWorldEnv(); });
        return spinBox;
    };

    // Time scale
    m_spinTimeScaleVal
            = createSliderBlock(QStringLiteral("Time scale"), 0.5, 10.0, 0.5, 1, initTimeScale);

    // Wind speed
    m_spinWindSpeedVal = createSliderBlock(
            QStringLiteral("Wind speed (m/s)"), 0.0, 30.0, 0.1, 1, initWindSpeed);

    // Wind up
    m_spinWindUpVal = createSliderBlock(
            QStringLiteral("Vertical up/down draft (m/s)"), -10.0, 10.0, 0.1, 1, initWindUp);

    // IR Contrast
    m_spinIrContrastVal
            = createSliderBlock(QStringLiteral("IR Contrast"), 0.0, 1010.0, 10.0, 0, 266.0, false);

    // GPS availability
    m_chkGpsOff = new QCheckBox(QStringLiteral("Emulate GPS signal unavailable"));
    m_chkGpsOff->setChecked(initGpsOff);
    connect(m_chkGpsOff, &QCheckBox::toggled, this, &GaiaWindow::sendWorldEnv);
    slidersLayout->addWidget(m_chkGpsOff, 0, Qt::AlignHCenter);

    mainLayout->addWidget(slidersFrame);

    // Right side: Dial (Angle Selector)
    QWidget *angleSelectorWidget = new QWidget();
    QVBoxLayout *angleSelectorLayout = new QVBoxLayout(angleSelectorWidget);

    QGridLayout *gridLayout = new QGridLayout();

    QLabel *lbl0 = new QLabel(QStringLiteral("0"));
    gridLayout->addWidget(lbl0, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignBottom);

    QLabel *lbl90 = new QLabel(QStringLiteral("90"));
    gridLayout->addWidget(lbl90, 1, 2, 1, 1, Qt::AlignLeft | Qt::AlignVCenter);

    QLabel *lbl270 = new QLabel(QStringLiteral("270"));
    gridLayout->addWidget(lbl270, 1, 0, 1, 1, Qt::AlignRight | Qt::AlignVCenter);

    QLabel *lbl180 = new QLabel(QStringLiteral("180"));
    gridLayout->addWidget(lbl180, 2, 1, 1, 1, Qt::AlignHCenter | Qt::AlignTop);

    m_dialWindDir = new ArrowDial();
    m_dialWindDir->setRange(0, 359);
    m_dialWindDir->setValue(static_cast<int>(initWindDir));
    QSizePolicy sp(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    sp.setHorizontalStretch(0);
    sp.setVerticalStretch(0);
    m_dialWindDir->setSizePolicy(sp);

    gridLayout->addWidget(m_dialWindDir, 1, 1, 1, 1);
    angleSelectorLayout->addLayout(gridLayout);

    QHBoxLayout *spinBoxLayout = new QHBoxLayout();
    QLabel *spinBoxLabel = new QLabel(QStringLiteral("Wind direction (°)"));
    spinBoxLayout->addWidget(spinBoxLabel);

    QSpinBox *spinBox = new QSpinBox();
    spinBox->setWrapping(true);
    spinBox->setRange(0, 359);
    spinBox->setValue(static_cast<int>(initWindDir));
    spinBoxLayout->addWidget(spinBox);

    angleSelectorLayout->addLayout(spinBoxLayout);

    connect(m_dialWindDir, &QDial::valueChanged, spinBox, &QSpinBox::setValue);
    connect(spinBox, &QSpinBox::valueChanged, m_dialWindDir, &QDial::setValue);
    connect(m_dialWindDir, &QDial::valueChanged, this, &GaiaWindow::sendWorldEnv);

    mainLayout->addWidget(angleSelectorWidget);

    // resize(500, 300);
    setCentralWidget(central);
}

void GaiaWindow::setupIvy(const QString &ivyBus)
{
    QString phome = qgetenv("PAPARAZZI_HOME");
    if (phome.isEmpty()) {
        phome = QDir::homePath() + QStringLiteral("/paparazzi");
    }
    QString xmlPath = phome + QStringLiteral("/var/messages.xml");

    if (!QFile::exists(xmlPath)) {
        qWarning() << "Gaia: message dictionary not found at" << xmlPath
                   << ". Ivy telemetry disabled.";
        return;
    }

    try {
        m_dict = std::make_unique<pprzlink::MessageDictionary>(xmlPath);
        m_link = std::make_unique<pprzlink::IvyQtLink>(*m_dict, "gaia", this);
        try {
            m_worldEnvDef = m_dict->getDefinition(QStringLiteral("WORLD_ENV"));
            m_defFound = true;
        } catch (...) {
            qWarning() << "Gaia: WORLD_ENV message not found in dictionary";
        }
        m_link->start(ivyBus);

        // Listen for WORLD_ENV_REQ and respond. This mirrors `Ground_Pprz.message_answerer my_id
        // "WORLD_ENV"`
        std::vector<pprzlink::MessageDefinition> defs
                = m_dict->getMsgsForClass(QStringLiteral("ground"));
        for (const auto &def : defs) {
            if (def.getName() == QLatin1String("WORLD_ENV_REQ")) {
                m_link->BindMessage(
                        def, this, [this](const QString &sender, const pprzlink::Message &msg) {
                            Q_UNUSED(sender);
                            Q_UNUSED(msg);
                            this->sendWorldEnv();
                        });
                break;
            }
        }
    } catch (const std::exception &e) {
        qWarning() << "Gaia: Exception during Ivy init:" << e.what();
    }
}

void GaiaWindow::sendWorldEnv()
{
    if (!m_link || !m_dict || !m_defFound)
        return;

    try {
        pprzlink::Message msg(m_worldEnvDef);
        msg.setSenderId("gaia");

        // Explicitly clamp all values to prevent any out-of-bounds telemetry injection
        double windSpeed = std::clamp(m_spinWindSpeedVal->value(), 0.0, 30.0);

        // Normalize wind direction to 0-359 degrees
        double windDirDeg = m_dialWindDir->value();
        windDirDeg = std::fmod(windDirDeg, 360.0);
        if (windDirDeg < 0)
            windDirDeg += 360.0;
        double windDirRad = windDirDeg * M_PI / 180.0;

        // Calculate velocity vectors
        double windEast = -windSpeed * sin(windDirRad);
        double windNorth = -windSpeed * cos(windDirRad);

        // Clamp vertical updraft
        double windUp = std::clamp(m_spinWindUpVal->value(), -10.0, 10.0);

        // Clamp hardware settings prevent any out-of-bounds telemetry
        float irContrast
                = static_cast<float>(std::clamp(m_spinIrContrastVal->value(), 0.0, 1010.0));
        float timeScale = static_cast<float>(std::clamp(m_spinTimeScaleVal->value(), 0.5, 10.0));
        uint8_t gpsAvail = m_chkGpsOff->isChecked() ? 0 : 1;

        // Try mapping the verified fields securely to the Ivy schema
        msg.addField(QStringLiteral("wind_east"), static_cast<float>(windEast));
        msg.addField(QStringLiteral("wind_north"), static_cast<float>(windNorth));
        msg.addField(QStringLiteral("wind_up"), static_cast<float>(windUp));
        msg.addField(QStringLiteral("ir_contrast"), irContrast);
        msg.addField(QStringLiteral("time_scale"), timeScale);
        msg.addField(QStringLiteral("gps_availability"), gpsAvail);

        m_link->sendMessage(msg);

    } catch (const std::exception &e) {
        qWarning() << "Gaia: failed to send WORLD_ENV:" << e.what();
    }
}

/**
 * @brief Application entry point.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status code.
 *
 * @details Pre-configures the XDG desktop environment hooks, sets up argument parameters
 * (including safe space-character concatenation loops), and triggers the main Qt event loop.
 */
int main(int argc, char *argv[])
{
    // Set metadata BEFORE application instantiation to prevent XDG portal double-registration
    // root cause ("Connection already associated with an application ID").
    QCoreApplication::setApplicationVersion(QStringLiteral(PPRZ_VERSION_DESC));
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_gaia"));
    QCoreApplication::setApplicationName(QStringLiteral("Gaia"));

    QApplication app(argc, argv);

    QString iconPath = QStringLiteral(":/penguin_icon_sim.png");
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(QApplication::desktopFileName(),
                                   QStringLiteral("Paparazzi Gaia"),
                                   QStringLiteral("World environment simulator"),
                                   iconPath,
                                   QStringLiteral("paparazzi-gaia"));
    QApplication::setWindowIcon(icon);
    QApplication::setStyle(new EditorLighteningStyle(QApplication::style()));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Paparazzi Gaia simulator component"));
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption busOption(QStringList() << QStringLiteral("b"),
                                 QStringLiteral("Ivy Bus (Default: 127.255.255.255:2010)"),
                                 QStringLiteral("bus"),
                                 QStringLiteral("127.255.255.255:2010"));
    parser.addOption(busOption);
    QCommandLineOption timeOption(QStringList() << QStringLiteral("t"),
                                  QStringLiteral("Set time scale (default: 1.0)"),
                                  QStringLiteral("timeScale"),
                                  QStringLiteral("1.0"));
    parser.addOption(timeOption);
    QCommandLineOption windSpeedOption(QStringList() << QStringLiteral("w"),
                                       QStringLiteral("Set wind speed (0-30m/s)"),
                                       QStringLiteral("windSpeed"),
                                       QStringLiteral("0.0"));
    parser.addOption(windSpeedOption);
    QCommandLineOption windDirOption(QStringList() << QStringLiteral("d"),
                                     QStringLiteral("Set wind direction 0-359 deg"),
                                     QStringLiteral("windDir"),
                                     QStringLiteral("0.0"));
    parser.addOption(windDirOption);
    QCommandLineOption windUpOption(QStringList() << QStringLiteral("u"),
                                    QStringLiteral("Set wind vertical draft (-10 to 10m/s)"),
                                    QStringLiteral("windUp"),
                                    QStringLiteral("0.0"));
    parser.addOption(windUpOption);
    QCommandLineOption gpsOffOption(
            QStringList() << QStringLiteral("g"),
            QStringLiteral("Emulate GPS signal unavailable (default: false)"));
    parser.addOption(gpsOffOption);
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

    QString ivyBus = parser.value(busOption);
    double timeScale = parser.value(timeOption).toDouble();
    double windSpeed = parser.value(windSpeedOption).toDouble();
    double windDir = parser.value(windDirOption).toDouble();
    double windUp = parser.value(windUpOption).toDouble();
    bool gpsOff = parser.isSet(gpsOffOption);

    GaiaWindow window(ivyBus, timeScale, windSpeed, windDir, windUp, gpsOff);
    window.show();

    return QApplication::exec();
}

#include "gaia.moc"
