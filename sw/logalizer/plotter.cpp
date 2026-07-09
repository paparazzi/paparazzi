/**
 * @file plotter.cpp
 * @brief Real-time telemetry plotter application for Paparazzi UAV.
 * @details This application connects to the Paparazzi Ivy bus, parses telemetry
 *          messages dynamically, and visualizes the data via Qt Charts in real-time.
 *          It is designed to be highly robust and memory-safe for long-running operations.
 */

#include <QAction>
#include <QApplication>
#include <QChart>
#include <QChartView>
#include <QCheckBox>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDateTime>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFile>
#include <QFrame>
#include <QGraphicsLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QIcon>
#include <QIODevice>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QLineSeries>
#include <QList>
#include <QMainWindow>
#include <QMargins>
#include <QMenu>
#include <QMenuBar>
#include <QObject>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPointF>
#include <QRegularExpression>
#include <QResizeEvent>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QValueAxis>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <mutex>
#include <utility>

#include "../include/os_desktop_utils.h"
#include "../include/pprz_version.h"
#include "plotter_common.h"

#include "pprzlinkQt/IvyQtLink.h"

/**
 * @struct PlotConfig
 * @brief Configuration and runtime state for an actively plotted curve.
 * @details This structure bundles the telemetry metadata (sender, message name, field)
 *          with the Qt rendering components (QLineSeries, action menus) and an internal
 *          buffer. Buffering points incoming rapidly prevents excessive redraw calls
 *          on the QChart side, allowing high update rates gracefully.
 */
struct PlotConfig
{
    QString senderName;
    QRegularExpression senderNameRegex;
    bool hasWildcard = false;
    QString className;
    QString msgName;
    QString fieldName;
    double coef;
    QLineSeries *series;
    QList<QPointF> history; // Master absolute timestamps tracking points
    QList<QPointF> buffer;
    int fieldIndex = -1;
    bool discrete = false;
    QAction *avgAction = nullptr;
    QAction *stdevAction = nullptr;
    int arrayIndex = -1;
};

/**
 * @class PlotterWindow
 * @brief The main GUI manager encompassing the plotting canvas, tools, and Ivy messaging.
 * @details This class is responsible for spawning the main window layout, registering
 *          drag-and-drop operations for curve associations, processing parsed variables
 *          into chart instances, and maintaining boundary rules (auto-scaling, min/max limits).
 */
struct PlotterWindowConfig
{
    QString title;
    QString geometry;
    int memorySize = 500;
    double updateTime = 0.5;
    QStringList curves;
};

/**
 * @brief Represents the PlotterWindow class.
 * @details This class encapsulates the primary logic and UI structures required
 * for PlotterWindow operations, ensuring robust and memory-safe management within the telemetry
 * pipeline.
 */
class PlotterWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit PlotterWindow(const PlotterWindowConfig &config,
                           pprzlink::MessageDictionary *dict,
                           pprzlink::IvyQtLink *link,
                           QWidget *parent = nullptr);
    ~PlotterWindow() override;

protected:
    void resizeEvent(QResizeEvent *event) override;
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;

private:
    void onClearClicked();
    void onPauseToggled(bool checked);
    void onAutoScaleToggled(bool checked);
    void onManualScaleChanged();
    void onAddConstantClicked();
    void onUpdateRateChanged(int val);
    void onLineThicknessChanged(int val);
    void updatePlots();
    void onLegendRefreshTimeout();

    ChartLegendManager *m_legendManager;

    void setupUI();
    void setupMenu();
    void addPlotFromPayload(const QString &payload);
    void handleMessage(QString sender, const pprzlink::Message &msg);

    void addCurveToMenu(PlotConfig &cfg);
    void removeCurve(QLineSeries *series);
    void recalculateYBounds();

    QChart *m_chart;
    QValueAxis *m_axisX;
    QValueAxis *m_axisY;
    qint64 m_startTime;

    pprzlink::MessageDictionary *m_dict;
    pprzlink::IvyQtLink *m_link;

    QList<PlotConfig> m_activePlots;

    double m_minY;
    double m_maxY;
    bool m_paused {false};
    bool m_autoScale {true};

    QCheckBox *m_cbAutoScale;
    QLineEdit *m_edtMinY;
    QLineEdit *m_edtMaxY;
    QSlider *m_slTimeWindow;
    QLineEdit *m_edtConstant;
    QSlider *m_slUpdateRate;
    QLineEdit *m_edtScaleNext;
    QSpinBox *m_spnLineThickness;
    QTimer *m_updateTimer;
    QMenu *m_curvesMenu;
    QTimer *m_statsTimer;
    bool m_statsNeedsRefresh {false};
    std::recursive_mutex m_plotMutex;
};

/**
 * @brief Safely extracts a native double value from a generic pprzlink FieldValue.
 * @param value The strongly-typed variant value transmitted over the Ivy bus.
 * @return The converted 64-bit float, or a quiet NaN if the layout is an array or unparseable
 * string.
 * @details Ivy telemetry packages data in multiple raw binary types. This helper cleanly cascades
 *          downward through the supported type taxonomy to normalize inputs onto a
 * plotting-friendly 1D numerical axis.
 */
static double fieldValueAsDouble(const pprzlink::FieldValue &value, int arrayIndex = -1)
{
    const auto &type = value.getType();
    if (type.isArray()) {
        if (arrayIndex >= 0) {
            try {
                switch (type.getBaseType()) {
                case pprzlink::BaseType::CHAR: {
                    std::vector<char> v;
                    value.getValue(v);
                    if (static_cast<size_t>(arrayIndex) < v.size())
                        return static_cast<double>(v[arrayIndex]);
                } break;
                case pprzlink::BaseType::INT8: {
                    std::vector<int8_t> v;
                    value.getValue(v);
                    if (static_cast<size_t>(arrayIndex) < v.size())
                        return static_cast<double>(v[arrayIndex]);
                } break;
                case pprzlink::BaseType::INT16: {
                    std::vector<int16_t> v;
                    value.getValue(v);
                    if (static_cast<size_t>(arrayIndex) < v.size())
                        return static_cast<double>(v[arrayIndex]);
                } break;
                case pprzlink::BaseType::INT32: {
                    std::vector<int32_t> v;
                    value.getValue(v);
                    if (static_cast<size_t>(arrayIndex) < v.size())
                        return static_cast<double>(v[arrayIndex]);
                } break;
                case pprzlink::BaseType::UINT8: {
                    std::vector<uint8_t> v;
                    value.getValue(v);
                    if (static_cast<size_t>(arrayIndex) < v.size())
                        return static_cast<double>(v[arrayIndex]);
                } break;
                case pprzlink::BaseType::UINT16: {
                    std::vector<uint16_t> v;
                    value.getValue(v);
                    if (static_cast<size_t>(arrayIndex) < v.size())
                        return static_cast<double>(v[arrayIndex]);
                } break;
                case pprzlink::BaseType::UINT32: {
                    std::vector<uint32_t> v;
                    value.getValue(v);
                    if (static_cast<size_t>(arrayIndex) < v.size())
                        return static_cast<double>(v[arrayIndex]);
                } break;
                case pprzlink::BaseType::FLOAT: {
                    std::vector<float> v;
                    value.getValue(v);
                    if (static_cast<size_t>(arrayIndex) < v.size())
                        return static_cast<double>(v[arrayIndex]);
                } break;
                case pprzlink::BaseType::DOUBLE: {
                    std::vector<double> v;
                    value.getValue(v);
                    if (static_cast<size_t>(arrayIndex) < v.size())
                        return static_cast<double>(v[arrayIndex]);
                } break;
                default:
                    return std::numeric_limits<double>::quiet_NaN();
                }
            } catch (...) {
                // Malformed array element; report the value as unavailable.
                return std::numeric_limits<double>::quiet_NaN();
            }
        }
        return std::numeric_limits<double>::quiet_NaN();
    }
    switch (type.getBaseType()) {
    case pprzlink::BaseType::CHAR: {
        char v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::INT8: {
        int8_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::INT16: {
        int16_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::INT32: {
        int32_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::UINT8: {
        uint8_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::UINT16: {
        uint16_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::UINT32: {
        uint32_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::FLOAT: {
        float v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::DOUBLE: {
        double v;
        value.getValue(v);
        return v;
    }
    case pprzlink::BaseType::STRING: {
        QString s;
        value.getValue(s);
        bool ok = false;
        double d = s.toDouble(&ok);
        return ok ? d : std::numeric_limits<double>::quiet_NaN();
    }
    default:
        return std::numeric_limits<double>::quiet_NaN();
    }
}

/**
 * @brief Constructs the Plotter Window and instantiates all visual layouts.
 * @param parent Optional parent widget (usually null for root windows).
 * @details Establishes zero-margin frameless QChart setups, registers memory
 *          handling attributes like `WA_DeleteOnClose` to prevent leaks upon
 *          user dismissal, and wires up the UI actions.
 */
PlotterWindow::PlotterWindow(const PlotterWindowConfig &config,
                             pprzlink::MessageDictionary *dict,
                             pprzlink::IvyQtLink *link,
                             QWidget *parent)
    : QMainWindow(parent)
    , m_dict(dict)
    , m_link(link)
    , m_minY(std::numeric_limits<double>::infinity())
    , m_maxY(-std::numeric_limits<double>::infinity())
{
    m_legendManager = nullptr;
    setAcceptDrops(true);
    setAttribute(Qt::WA_DeleteOnClose);
    setWindowTitle(config.title.isEmpty() ? QStringLiteral("Plotter") : config.title);

    if (!config.geometry.isEmpty()) {
        QRegularExpression re(QStringLiteral(R"(^(\d+)x(\d+)(?:\+[-]?(\d+)\+[-]?(\d+))?$)"));
        QRegularExpressionMatch match = re.match(config.geometry);
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
            resize(900, 200);
        }
    } else {
        resize(900, 200);
    }

    m_chart = new QChart();
    m_chart->setTitle(QStringLiteral("Drag & Drop messages here"));
    m_chart->setAnimationOptions(QChart::NoAnimation);
    m_chart->setBackgroundRoundness(0);
    m_chart->setMargins(QMargins(0, 0, 0, 0));
    m_chart->layout()->setContentsMargins(0, 0, 0, 0);
    m_chart->setBackgroundPen(QPen(Qt::NoPen));

    m_axisX = new QValueAxis();
    m_axisX->setTitleText(QStringLiteral("Time (s)"));
    m_axisX->setTitleVisible(false); // Hidden by default
    // m_axisX->setLabelFormat("%gs"); previous if you want to
    m_axisX->setLabelFormat(QStringLiteral("%.1fs"));
    m_chart->addAxis(m_axisX, Qt::AlignBottom);

    m_axisY = new QValueAxis();
    m_axisY->setTitleText(QStringLiteral("Value"));
    m_axisY->setTitleVisible(false); // Hidden by default
    m_chart->addAxis(m_axisY, Qt::AlignLeft);

    setupUI();
    setupMenu();

    m_startTime = QDateTime::currentMSecsSinceEpoch();

    // Apply configuration values
    if (config.updateTime > 0) {
        int updateVal = static_cast<int>(config.updateTime * 100.0);
        m_slUpdateRate->setValue(
                std::clamp(updateVal, m_slUpdateRate->minimum(), m_slUpdateRate->maximum()));
    }
    if (config.memorySize > 0) {
        m_slTimeWindow->setValue(std::clamp(
                config.memorySize, m_slTimeWindow->minimum(), m_slTimeWindow->maximum()));
    }

    for (const QString &curve : config.curves) {
        addPlotFromPayload(curve);
    }
}

/**
 * @brief Destructor.
 * @details Windows no longer own m_link or m_dict; they are managed in main().
 */
PlotterWindow::~PlotterWindow() = default;

/**
 * @brief Constructs the application's widgets, layouts, menus, and timers dynamically.
 * @details This separates graphical state binding away from pure telemetry handling.
 *          Timers are generated here regulating UI FPS (default ~60Hz base).
 */
void PlotterWindow::setupUI()
{
    QWidget *mainWidget = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(mainWidget);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    QWidget *toolbarWidget = new QWidget();
    QHBoxLayout *toolbarLayout = new QHBoxLayout(toolbarWidget);
    toolbarLayout->setContentsMargins(2, 2, 2, 2); // keep some margin for the toolbar

    m_cbAutoScale = new QCheckBox(QStringLiteral("Auto Scale"));
    m_cbAutoScale->setChecked(true);

    m_edtMinY = new QLineEdit();
    m_edtMaxY = new QLineEdit();
    m_edtMinY->setMaximumWidth(60);
    m_edtMaxY->setMaximumWidth(60);
    m_edtMinY->setEnabled(false);
    m_edtMaxY->setEnabled(false);

    m_slTimeWindow = new QSlider(Qt::Horizontal);
    m_slTimeWindow->setToolTip(QStringLiteral("Memory Size"));
    m_slTimeWindow->setRange(10, 1000); // 10 to 1000 points
    m_slTimeWindow->setSingleStep(10);
    m_slTimeWindow->setPageStep(10);
    m_slTimeWindow->setValue(500); // Default to 500

    QLabel *lblConst = new QLabel(QStringLiteral("Constant"));
    m_edtConstant = new QLineEdit();
    m_edtConstant->setMaximumWidth(50);

    QLabel *lblScaleNext = new QLabel(QStringLiteral("Scale next by"));
    m_edtScaleNext = new QLineEdit(QStringLiteral("1.0"));
    m_edtScaleNext->setMaximumWidth(50);
    m_edtScaleNext->setToolTip(QStringLiteral(
            "Scale next curve (e.g. 0.0174 to convert deg in rad, 57.3 to convert rad in deg)"));

    m_slUpdateRate = new QSlider(Qt::Horizontal);
    m_slUpdateRate->setToolTip(QStringLiteral("Update Rate (s)"));
    m_slUpdateRate->setRange(5, 100); // 0.05s to 1.00s (multiplied by 100 for integer slider)
    m_slUpdateRate->setValue(50); // Default to 0.5s => 50

    m_spnLineThickness = new QSpinBox();
    m_spnLineThickness->setToolTip(QStringLiteral("Line Thickness (px)"));
    m_spnLineThickness->setRange(1, 10);
    m_spnLineThickness->setValue(1);
    m_spnLineThickness->hide();

    m_updateTimer = new QTimer(this);
    m_updateTimer->start(m_slUpdateRate->value() * 10);

    m_statsTimer = new QTimer(this);
    m_statsTimer->setInterval(200);
    connect(m_statsTimer, &QTimer::timeout, this, &PlotterWindow::onLegendRefreshTimeout);
    m_statsTimer->start();

    toolbarLayout->addWidget(m_cbAutoScale);
    toolbarLayout->addWidget(new QLabel(QStringLiteral("Min")));
    toolbarLayout->addWidget(m_edtMinY);
    toolbarLayout->addWidget(new QLabel(QStringLiteral("Max")));
    toolbarLayout->addWidget(m_edtMaxY);

    QLabel *lblTimeWindowVal = new QLabel(QStringLiteral("%1").arg(m_slTimeWindow->value()));
    lblTimeWindowVal->setAlignment(Qt::AlignCenter);
    lblTimeWindowVal->setToolTip(QStringLiteral("Memory Size"));
    QVBoxLayout *vboxTime = new QVBoxLayout();
    vboxTime->addWidget(lblTimeWindowVal);
    vboxTime->addWidget(m_slTimeWindow);
    vboxTime->setContentsMargins(0, 0, 0, 0);
    QWidget *wTime = new QWidget();
    wTime->setLayout(vboxTime);

    QLabel *lblUpdateRateVal
            = new QLabel(QStringLiteral("%1").arg(m_slUpdateRate->value() / 100.0, 0, 'f', 2));
    lblUpdateRateVal->setAlignment(Qt::AlignCenter);
    lblUpdateRateVal->setToolTip(QStringLiteral("Update Rate (s)"));
    QVBoxLayout *vboxRate = new QVBoxLayout();
    vboxRate->addWidget(lblUpdateRateVal);
    vboxRate->addWidget(m_slUpdateRate);
    vboxRate->setContentsMargins(0, 0, 0, 0);
    QWidget *wRate = new QWidget();
    wRate->setLayout(vboxRate);

    connect(m_slTimeWindow,
            &QSlider::valueChanged,
            lblTimeWindowVal,
            [this, lblTimeWindowVal](int val) {
                lblTimeWindowVal->setText(QStringLiteral("%1").arg(val));
                this->updatePlots(); // Instantly apply memory crop to the graph rendering
            });
    connect(m_slUpdateRate, &QSlider::valueChanged, lblUpdateRateVal, [lblUpdateRateVal](int val) {
        lblUpdateRateVal->setText(QStringLiteral("%1").arg(val / 100.0, 0, 'f', 2));
    });

    toolbarLayout->addWidget(wRate, 1);
    toolbarLayout->addWidget(wTime, 1);
    toolbarLayout->addWidget(lblConst);
    toolbarLayout->addWidget(m_edtConstant);
    toolbarLayout->addWidget(lblScaleNext);
    toolbarLayout->addWidget(m_edtScaleNext);
    QLabel *lblLineThickness = new QLabel(QStringLiteral("Line:"));
    lblLineThickness->hide();
    toolbarLayout->addWidget(lblLineThickness);
    toolbarLayout->addWidget(m_spnLineThickness);

    QChartView *chartView = new QChartView(m_chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setAcceptDrops(false); // Let drops fall through
    chartView->setContentsMargins(0, 0, 0, 0);
    chartView->setFrameShape(QFrame::NoFrame);
    chartView->setToolTip(tr("Drop a messages field here to draw it"));

    mainLayout->addWidget(toolbarWidget);
    mainLayout->addWidget(chartView);
    setCentralWidget(mainWidget);

    m_legendManager = new ChartLegendManager(m_chart, chartView);

    connect(m_cbAutoScale, &QCheckBox::toggled, this, &PlotterWindow::onAutoScaleToggled);
    connect(m_edtMinY, &QLineEdit::editingFinished, this, &PlotterWindow::onManualScaleChanged);
    connect(m_edtMaxY, &QLineEdit::editingFinished, this, &PlotterWindow::onManualScaleChanged);
    connect(m_edtConstant, &QLineEdit::editingFinished, this, &PlotterWindow::onAddConstantClicked);
    connect(m_slUpdateRate, &QSlider::valueChanged, this, &PlotterWindow::onUpdateRateChanged);
    connect(m_spnLineThickness,
            QOverload<int>::of(&QSpinBox::valueChanged),
            this,
            &PlotterWindow::onLineThicknessChanged);
    connect(m_updateTimer, &QTimer::timeout, this, &PlotterWindow::updatePlots);
}

/**
 * @brief Periodically syncs statistical computations directly to UI display labels.
 * @details Re-walking plot arrays repeatedly is intense; batching them via a gentle
 *          0.2s timer enables deep math analysis (StdDev, Average) without choking
 *          the fast-rendering path.
 */
void PlotterWindow::onLegendRefreshTimeout()
{
    if (!m_statsNeedsRefresh) {
        return;
    }
    m_statsNeedsRefresh = false;

    std::scoped_lock lock(m_plotMutex);
    // Compute average and standard deviation for each curve
    for (const auto &plot : std::as_const(m_activePlots)) {
        if (!plot.series || (!plot.avgAction && !plot.stdevAction))
            continue;
        int n = plot.history.size();
        if (n < 1) {
            if (plot.avgAction)
                plot.avgAction->setText(tr("Average: N/A"));
            if (plot.stdevAction)
                plot.stdevAction->setText(tr("Stdev: N/A"));
            continue;
        }
        double sum = 0.0;
        double sum_sq = 0.0;
        for (int i = 0; i < n; ++i) {
            double y = plot.history.at(i).y();
            sum += y;
            sum_sq += y * y;
        }
        double fn = static_cast<double>(n);
        double avg = sum / fn;

        if (plot.avgAction) {
            plot.avgAction->setText(QStringLiteral("Average: %1").arg(avg, 0, 'f', 6));
        }

        if (plot.stdevAction) {
            if (n < 2) {
                plot.stdevAction->setText(tr("Stdev: N/A"));
            } else {
                double variance = (sum_sq - fn * avg * avg) / fn;
                double stdev = (variance > 0.0) ? std::sqrt(variance) : 0.0;
                plot.stdevAction->setText(QStringLiteral("Stdev: %1").arg(stdev, 0, 'f', 6));
            }
        }
    }
}

/**
 * @brief Obliterates all existing series data, purging memory completely.
 * @details Resets internal tracking extrema and the unified time origin safely.
 */
void PlotterWindow::onClearClicked()
{
    std::scoped_lock lock(m_plotMutex);
    for (auto &plot : m_activePlots) {
        if (plot.series) {
            m_chart->removeSeries(plot.series);
            delete plot.series;
            plot.series = nullptr;
        }
        plot.history.clear();
        plot.buffer.clear();
    }
    m_activePlots.clear();
    if (m_curvesMenu) {
        m_curvesMenu->clear();
    }
    m_chart->setTitle(QStringLiteral("Drag & Drop fields here"));
    m_minY = std::numeric_limits<double>::infinity();
    m_maxY = -std::numeric_limits<double>::infinity();
    m_startTime = QDateTime::currentMSecsSinceEpoch(); // reset time origin
    m_statsNeedsRefresh = true;
    // remove constants too if any
}

/**
 * @brief Freezes background updates gracefully. Buffer continues, but UI slumbers.
 */
void PlotterWindow::onPauseToggled(bool checked)
{
    m_paused = checked;
}

/**
 * @brief Flips mode constraints when toggling automated y-axis boundaries.
 */
void PlotterWindow::onAutoScaleToggled(bool checked)
{
    m_autoScale = checked;
    m_edtMinY->setEnabled(!checked);
    m_edtMaxY->setEnabled(!checked);
    if (!checked) {
        onManualScaleChanged();
    } else {
        recalculateYBounds();
    }
}

/**
 * @brief Manual override parsing strictly for user text entries dictating min/max ranges.
 */
void PlotterWindow::onManualScaleChanged()
{
    if (!m_autoScale) {
        bool okMin = false;
        bool okMax = false;
        double minY = m_edtMinY->text().toDouble(&okMin);
        double maxY = m_edtMaxY->text().toDouble(&okMax);
        if (okMin && okMax && minY < maxY) {
            m_axisY->setRange(minY, maxY);
        }
    }
}

/**
 * @brief Recalculates mathematical bounds precisely across all visual layers (DRY implementation).
 * @details Because series logic actively drops historical buffers natively as sliding Windows
 *          pass by, computing bounds sequentially against only the surviving active frame
 * guarantees clean structural resizing across multi-curves seamlessly without duplicating array
 * traversal.
 */
void PlotterWindow::recalculateYBounds()
{
    if (!m_autoScale)
        return;

    std::scoped_lock lock(m_plotMutex);
    m_minY = std::numeric_limits<double>::infinity();
    m_maxY = -std::numeric_limits<double>::infinity();
    bool hasPoints = false;

    for (const auto &plot : std::as_const(m_activePlots)) {
        if (!plot.series)
            continue;
        for (auto pt : plot.history) {
            m_minY = std::min(pt.y(), m_minY);
            m_maxY = std::max(pt.y(), m_maxY);
            hasPoints = true;
        }
    }

    if (hasPoints && m_minY <= m_maxY) {
        double margin = (m_maxY - m_minY) * 0.1;
        if (margin == 0.0)
            margin = 1.0;
        m_axisY->setRange(m_minY - margin, m_maxY + margin);
        m_edtMinY->setText(QString::number(m_minY - margin, 'f', 2));
        m_edtMaxY->setText(QString::number(m_maxY + margin, 'f', 2));
    }
}

/**
 * @brief Injects a static infinite-length visual baseline dynamically.
 * @details Synthesizes a faux PlotConfig that bypasses network hooks but renders evenly
 *          across the whole epoch timeline acting as a visual ruler.
 */
void PlotterWindow::onAddConstantClicked()
{
    bool ok = false;
    double val = m_edtConstant->text().toDouble(&ok);
    if (!ok || !std::isfinite(val))
        return;

    std::scoped_lock lock(m_plotMutex);

    // Create a new constant series
    PlotConfig cfg;
    cfg.senderName = QStringLiteral("sys");
    cfg.className = QStringLiteral("const");
    cfg.msgName = QStringLiteral("const");
    cfg.fieldName = QStringLiteral("C=%1").arg(val);
    cfg.coef = 1.0;

    cfg.series = new QLineSeries();
    cfg.series->setName(cfg.fieldName);

    // Assign black color for constant lines
    QPen pen1 = cfg.series->pen();
    pen1.setColor(Qt::black);
    pen1.setWidth(m_spnLineThickness->value());
    cfg.series->setPen(pen1);
    m_chart->addSeries(cfg.series);

    // Series points will populate robustly on the first handleMessage payload,
    // or continuously from updatePlots() geometry-shaping if this is a 'const'.

    cfg.series->attachAxis(m_axisX);
    cfg.series->attachAxis(m_axisY);

    m_activePlots.append(cfg);

    addCurveToMenu(m_activePlots.last());
    QTimer::singleShot(10, m_legendManager, &ChartLegendManager::updateLegendPosition);
}

/**
 * @brief Dynamic slider hook propagating rendering speeds natively down to QTimer intervals.
 */
void PlotterWindow::onUpdateRateChanged(int val)
{
    const int interval = std::max(10, val * 10);
    // Restart the timer so it immediately adapts to the new interval
    m_updateTimer->start(interval);

    // Smoothly redraw to instantly clamp or expand the visible time window
    updatePlots();
}

/**
 * @brief Permits receiving raw text drops from other X11/Wayland Desktop apps.
 */
void PlotterWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasText()) {
        event->acceptProposedAction();
    }
}

/**
 * @brief Orchestrates raw dropped strings into formal telemetry series requests.
 */
class DelayedDropWatcher : public QObject
{
    Q_OBJECT
    QString m_filePath;
    QTimer *m_timer;
    int m_attempts {0};

public:
    DelayedDropWatcher(const QString &path, QObject *parent)
        : QObject(parent)
        , m_filePath(path)
    {
        m_timer = new QTimer(this);
        connect(m_timer, &QTimer::timeout, this, &DelayedDropWatcher::checkFile);
        m_timer->start(100);
    }
signals:
    void payloadsReady(const QString &);
private slots:
    void checkFile()
    {
        m_attempts++;
        QFile f(m_filePath);
        if (f.exists()) {
            if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
                QString content = f.readAll();
                f.close();
                f.remove();
                if (!content.isEmpty()) {
                    emit payloadsReady(content);
                }
                m_timer->stop();
                deleteLater();
            }
        } else if (m_attempts > 600) { // 60 seconds timeout
            m_timer->stop();
            deleteLater();
        }
    }
};

void PlotterWindow::dropEvent(QDropEvent *event)
{
    if (event->mimeData()->hasText()) {
        QString payloadText = event->mimeData()->text();
        if (payloadText.startsWith(QLatin1String("delayed_array:"))) {
            QString filePath = payloadText.mid(14);
            DelayedDropWatcher *watcher = new DelayedDropWatcher(filePath, this);
            connect(watcher,
                    &DelayedDropWatcher::payloadsReady,
                    this,
                    [this](const QString &content) {
                        QStringList payloads = content.split('\n', Qt::SkipEmptyParts);
                        for (const QString &payload : std::as_const(payloads)) {
                            this->addPlotFromPayload(payload);
                        }
                    });
            event->acceptProposedAction();
            return;
        }

        // payloadText can contain multiple payloads separated by newline
        QStringList payloads = payloadText.split('\n', Qt::SkipEmptyParts);
        for (const QString &payload : std::as_const(payloads)) {
            addPlotFromPayload(payload);
        }
        event->acceptProposedAction();
    }
}

/**
 * @brief Translates structured Paparazzi textual signatures into hard-linked data curves.
 * @param payload E.g. "senderName:className:msgName:fieldName:optional_coef".
 * @details Establishes a formal Ivy-bus lambda subscription parsing the specific
 *          index directly matching the field name required natively upon connection.
 */
void PlotterWindow::addPlotFromPayload(const QString &payload)
{
    // payload format: m_senderName + ":" + m_className + ":" + msgName + ":" + fieldName + ":" +
    // coef;
    QStringList parts = payload.split(QStringLiteral(":"));
    if (parts.size() >= 4) {
        PlotConfig cfg;
        cfg.senderName = parts[0];
        if (cfg.senderName.contains('*') || cfg.senderName.contains('?')) {
            cfg.hasWildcard = true;
            cfg.senderNameRegex.setPattern(
                    QRegularExpression::wildcardToRegularExpression(cfg.senderName));
        }
        cfg.className = parts[1];
        cfg.msgName = parts[2];

        const QString &fieldStr = parts[3];
        int bracketIndex = fieldStr.indexOf('[');
        if (bracketIndex != -1 && fieldStr.endsWith(']')) {
            cfg.fieldName = fieldStr.left(bracketIndex);
            cfg.arrayIndex = QStringView(fieldStr)
                                     .mid(bracketIndex + 1, fieldStr.length() - bracketIndex - 2)
                                     .toInt();
        } else {
            cfg.fieldName = fieldStr;
            cfg.arrayIndex = -1;
        }

        bool ok = false;
        double scaleNext = m_edtScaleNext->text().toDouble(&ok);
        if (!ok || !std::isfinite(scaleNext))
            scaleNext = 1.0;
        double coef = 1.0;
        if (parts.size() >= 5) {
            bool okCoef = false;
            coef = parts[4].toDouble(&okCoef);
            if (!okCoef || !std::isfinite(coef))
                coef = 1.0;
        }
        cfg.coef = coef * scaleNext;
        if (cfg.coef == 0.0 || !std::isfinite(cfg.coef))
            cfg.coef = 1.0;

        // Check if already plotted
        for (const auto &existing : std::as_const(m_activePlots)) {
            if (existing.senderName == cfg.senderName && existing.msgName == cfg.msgName
                && existing.fieldName == cfg.fieldName && existing.arrayIndex == cfg.arrayIndex) {
                return; // already plotting
            }
        }

        cfg.series = new QLineSeries();
        QString prefix = (cfg.senderName.isEmpty() || cfg.senderName == QLatin1String("all"))
                ? QLatin1String("")
                : cfg.senderName + QStringLiteral(":");
        QString classPrefix
                = cfg.className.isEmpty() ? QLatin1String("") : cfg.className + QStringLiteral(":");
        QString arraySuffix = (cfg.arrayIndex >= 0) ? QStringLiteral("[%1]").arg(cfg.arrayIndex)
                                                    : QLatin1String("");
        cfg.series->setName(
                QStringLiteral("%1%2%3:%4%5")
                        .arg(prefix, classPrefix, cfg.msgName, cfg.fieldName, arraySuffix));

        // Assign custom distinct saturated color
        QPen pen2 = cfg.series->pen();
        pen2.setColor(getNextSaturatedColor());
        pen2.setWidth(m_spnLineThickness->value());
        cfg.series->setPen(pen2);
        m_chart->addSeries(cfg.series);
        cfg.series->attachAxis(m_axisX);
        cfg.series->attachAxis(m_axisY);

        int fieldIndex = -1;
        bool alreadyBound = false;
        std::vector<pprzlink::MessageDefinition> defs;
        if (m_dict) {
            defs = m_dict->getMsgsForClass(cfg.className);
        }
        for (const auto &existing : std::as_const(m_activePlots)) {
            if (existing.msgName == cfg.msgName && existing.className == cfg.className) {
                alreadyBound = true;
                break;
            }
        }
        for (const auto &def : defs) {
            if (def.getName() == cfg.msgName) {
                for (int k = 0; k < (int) def.getNbFields(); ++k) {
                    if (def.getField(k).getName() == cfg.fieldName) {
                        fieldIndex = k;
                        break;
                    }
                }
                if (!alreadyBound && m_link) {
                    // qDebug() << "Binding message:" << cfg.msgName;
                    m_link->BindMessage(
                            def, this, [this](const QString &sender, const pprzlink::Message &msg) {
                                this->handleMessage(sender, msg);
                            });
                }
                break;
            }
        }
        cfg.fieldIndex = fieldIndex;

        {
            std::scoped_lock lock(m_plotMutex);
            m_activePlots.append(cfg);
            m_chart->setTitle(QLatin1String(""));

            addCurveToMenu(m_activePlots.last());
        }
        m_statsNeedsRefresh = true;
        QTimer::singleShot(10, m_legendManager, &ChartLegendManager::updateLegendPosition);
    }
}

/**
 * @brief Bootstraps standard menubar hooks supporting application-level suspension/quit calls.
 */
void PlotterWindow::setupMenu()
{
    QMenu *plotMenu = menuBar()->addMenu(tr("&Plot"));
    plotMenu->setToolTipsVisible(true);

    QAction *newAction = plotMenu->addAction(tr("New"));
    newAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+N")));
    connect(newAction, &QAction::triggered, this, [this]() {
        PlotterWindow *newWindow = new PlotterWindow(PlotterWindowConfig(), m_dict, m_link);
        newWindow->show();
    });

    QAction *resetAction = plotMenu->addAction(tr("Reset"));
    resetAction->setToolTip(tr("Reset the current display and the current data"));
    resetAction->setStatusTip(tr("Reset the current display and the current data"));
    resetAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+L")));
    connect(resetAction, &QAction::triggered, this, &PlotterWindow::onClearClicked);

    QAction *suspendAction = plotMenu->addAction(tr("Suspend"));
    suspendAction->setToolTip(tr("Freeze the display while the data are still updated"));
    suspendAction->setStatusTip(tr("Freeze the display while the data are still updated"));
    suspendAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+S")));
    connect(suspendAction, &QAction::triggered, this, [this]() { m_paused = true; });

    QAction *stopAction = plotMenu->addAction(tr("Stop"));
    stopAction->setToolTip(
            tr("Freeze the data update while the display is active (e.g. resizable)"));
    stopAction->setStatusTip(
            tr("Freeze the data update while the display is active (e.g. resizable)"));
    stopAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+C")));
    connect(stopAction, &QAction::triggered, this, [this]() { m_paused = true; });

    QAction *restartAction = plotMenu->addAction(tr("Restart"));
    restartAction->setToolTip(tr("UnFreeze"));
    restartAction->setStatusTip(tr("UnFreeze"));
    restartAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+X")));
    connect(restartAction, &QAction::triggered, this, [this]() { m_paused = false; });

    plotMenu->addSeparator();

    QAction *closeAction = plotMenu->addAction(tr("Close"));
    closeAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+W")));
    connect(closeAction, &QAction::triggered, this, &PlotterWindow::onClearClicked);

    QAction *quitAction = plotMenu->addAction(tr("Quit"));
    quitAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+Q")));
    connect(quitAction, &QAction::triggered, qApp, &QApplication::quit);

    m_curvesMenu = menuBar()->addMenu(tr("&Curves"));
}

/**
 * @brief Thread-safe ingestion queue receiving highly asynchronous Ivy bus telemetry packages.
 * @param sender Originating entity emitting the message format.
 * @param msg Validated and natively inflated binary structure definition mapping payload contents.
 * @details Appends numerical coordinates natively formatted into an internal memory buffer.
 *          We explicitly restrict raw redraw actions (`series->append`) here, delegating drawing
 *          solely to the main GUI event loop syncing logic securely to 60fps refresh limits.
 */
void PlotterWindow::handleMessage(QString sender, const pprzlink::Message &msg)
{
    if (m_paused)
        return; // Skip updating data while paused

    QString sId = std::move(sender);
    if (sId.isEmpty()) {
        const auto &senderV = msg.getSenderId();
        if (std::holds_alternative<QString>(senderV))
            sId = std::get<QString>(senderV);
        else
            sId = QString::number(std::get<uint8_t>(senderV));
    }
    if (sId.isEmpty())
        sId = QStringLiteral("ground");

    QString msgName = msg.getDefinition().getName();
    double currentTime = (QDateTime::currentMSecsSinceEpoch() - m_startTime) / 1000.0;

    std::scoped_lock lock(m_plotMutex);

    for (auto &plot : m_activePlots) {
        if (!plot.series)
            continue;

        bool senderMatches = false;
        if (plot.senderName.isEmpty() || plot.senderName == QLatin1String("all")
            || plot.senderName == QLatin1String("*")) {
            senderMatches = true;
        } else if (plot.hasWildcard) {
            senderMatches = plot.senderNameRegex.match(sId).hasMatch();
        } else {
            senderMatches = (plot.senderName == sId);
        }

        if (plot.msgName == msgName && senderMatches) {
            const auto &def = msg.getDefinition();
            int fieldIndex = plot.fieldIndex;
            if (fieldIndex < 0 || fieldIndex >= (int) def.getNbFields()) {
                // fallback path for legacy configs or missing cached index
                for (int i = 0; i < (int) def.getNbFields(); ++i) {
                    if (def.getField(i).getName() == plot.fieldName) {
                        fieldIndex = i;
                        plot.fieldIndex = i;
                        break;
                    }
                }
            }
            if (fieldIndex < 0 || fieldIndex >= (int) def.getNbFields()) {
                continue;
            }
            try {
                const auto &rv = msg.getRawValue(fieldIndex);
                if (rv.getType().isArray() && plot.arrayIndex == -1) {
                    try {
                        int arrSize = 0;
                        switch (rv.getType().getBaseType()) {
                        case pprzlink::BaseType::CHAR: {
                            std::vector<char> v;
                            rv.getValue(v);
                            arrSize = v.size();
                        } break;
                        case pprzlink::BaseType::INT8: {
                            std::vector<int8_t> v;
                            rv.getValue(v);
                            arrSize = v.size();
                        } break;
                        case pprzlink::BaseType::INT16: {
                            std::vector<int16_t> v;
                            rv.getValue(v);
                            arrSize = v.size();
                        } break;
                        case pprzlink::BaseType::INT32: {
                            std::vector<int32_t> v;
                            rv.getValue(v);
                            arrSize = v.size();
                        } break;
                        case pprzlink::BaseType::UINT8: {
                            std::vector<uint8_t> v;
                            rv.getValue(v);
                            arrSize = v.size();
                        } break;
                        case pprzlink::BaseType::UINT16: {
                            std::vector<uint16_t> v;
                            rv.getValue(v);
                            arrSize = v.size();
                        } break;
                        case pprzlink::BaseType::UINT32: {
                            std::vector<uint32_t> v;
                            rv.getValue(v);
                            arrSize = v.size();
                        } break;
                        case pprzlink::BaseType::FLOAT: {
                            std::vector<float> v;
                            rv.getValue(v);
                            arrSize = v.size();
                        } break;
                        case pprzlink::BaseType::DOUBLE: {
                            std::vector<double> v;
                            rv.getValue(v);
                            arrSize = v.size();
                        } break;
                        default:
                            break;
                        }
                        if (arrSize > 0) {
                            for (int k = 0; k < arrSize; ++k) {
                                QString subPayload = QStringLiteral("%1:%2:%3:%4[%5]:%6")
                                                             .arg(plot.senderName,
                                                                  plot.className,
                                                                  plot.msgName,
                                                                  plot.fieldName)
                                                             .arg(k)
                                                             .arg(plot.coef);
                                QMetaObject::invokeMethod(
                                        this,
                                        [this, subPayload]() {
                                            this->addPlotFromPayload(subPayload);
                                        },
                                        Qt::QueuedConnection);
                            }
                            QMetaObject::invokeMethod(
                                    this,
                                    [this, series = plot.series]() { this->removeCurve(series); },
                                    Qt::QueuedConnection);
                        }
                    } catch (...) {
                        // Best-effort array auto-expansion; skip this plot on failure.
                        qWarning()
                                << "Plotter: failed to auto-expand array field" << plot.fieldName;
                    }
                    continue;
                }

                double val = fieldValueAsDouble(rv, plot.arrayIndex);
                if (!std::isfinite(val))
                    continue;
                val *= plot.coef;

                if (plot.discrete) {
                    double lastY = val;
                    bool hasLastY = false;
                    if (!plot.buffer.isEmpty()) {
                        lastY = plot.buffer.last().y();
                        hasLastY = true;
                    } else if (plot.history.count() > 0) {
                        lastY = plot.history.last().y();
                        hasLastY = true;
                    }
                    // Optimize step-functions: only inject the right-angle corner if the state
                    // actually changed.
                    if (hasLastY && lastY != val) {
                        plot.buffer.append(QPointF(currentTime, lastY));
                    }
                }

                plot.buffer.append(QPointF(currentTime, val));
            } catch (const std::exception &e) {
                qWarning() << "Exception in handleMessage:" << e.what();
            } catch (...) {
                qWarning() << "Unknown exception in handleMessage.";
            }
        }
    }
}

/**
 * @brief Core 60Hz rendering pass draining back-buffers flushing into native widget views.
 * @details Modifies visual ranges by stripping natively expired history points mathematically off
 *          the time window scale (X-axis). Performs localized Y bounds expansion directly as
 *          arrays stream safely across active timeframes. Employs `std::isfinite` to guard
 *          against QChart canvas corruptions safely.
 */
void PlotterWindow::updatePlots()
{
    if (m_paused)
        return;

    double currentTime = (QDateTime::currentMSecsSinceEpoch() - m_startTime) / 1000.0;
    // OCaml total time window corresponds to (Memory Size) * (Update Time)
    double windowSize = m_slTimeWindow->value() * (m_slUpdateRate->value() / 100.0);
    bool needsAxisUpdate = false;
    bool needsFullRecalc = false;

    std::scoped_lock lock(m_plotMutex);

    for (auto &plot : m_activePlots) {
        if (!plot.series)
            continue;
        if (plot.className == QLatin1String("const")) {
            bool ok = false;
            double val = plot.fieldName.section('=', 1).toDouble(&ok);
            if (!ok || !std::isfinite(val))
                continue;
            plot.series->replace(QList<QPointF>() << QPointF(currentTime - windowSize, val)
                                                  << QPointF(currentTime, val));
            if (m_autoScale) {
                if (val < m_minY) {
                    m_minY = val;
                    needsAxisUpdate = true;
                }
                if (val > m_maxY) {
                    m_maxY = val;
                    needsAxisUpdate = true;
                }
            }
            continue;
        }

        if (!plot.buffer.isEmpty()) {
            plot.history.append(plot.buffer);
            for (const QPointF &pt : std::as_const(plot.buffer)) {
                if (pt.y() < m_minY) {
                    m_minY = pt.y();
                    needsAxisUpdate = true;
                }
                if (pt.y() > m_maxY) {
                    m_maxY = pt.y();
                    needsAxisUpdate = true;
                }
            }
            plot.buffer.clear();
        }

        int pointsToRemove = 0;
        int count = plot.history.count();
        double cutoffTime = currentTime - windowSize;
        while (pointsToRemove < count && plot.history.at(pointsToRemove).x() < cutoffTime) {
            double ptY = plot.history.at(pointsToRemove).y();
            // If the point we're dropping defined the bounding box, we must shrink/re-evaluate the
            // whole box natively.
            if (m_autoScale && (ptY <= m_minY || ptY >= m_maxY)) {
                needsFullRecalc = true;
            }
            pointsToRemove++;
        }
        if (pointsToRemove > 0) {
            plot.history.remove(0, pointsToRemove);
        }

        plot.series->replace(plot.history);
    }

    // Anchor X-axis such that right is current time and left is past (-windowSize)
    m_axisX->setRange(currentTime - windowSize, currentTime);

    if (m_autoScale) {
        if (needsFullRecalc) {
            recalculateYBounds();
        } else if (needsAxisUpdate && m_minY <= m_maxY) {
            double margin = (m_maxY - m_minY) * 0.1;
            if (margin == 0)
                margin = 1.0;
            m_axisY->setRange(m_minY - margin, m_maxY + margin);
            m_edtMinY->setText(QString::number(m_minY - margin, 'f', 2));
            m_edtMaxY->setText(QString::number(m_maxY + margin, 'f', 2));
        }
    }
    m_statsNeedsRefresh = true;
}

/**
 * @brief Automates drop-down bindings allocating math operators / deletion tools onto curves.
 * @param cfg Passed dynamically to tether UI state triggers directly towards struct instances.
 */
void PlotterWindow::addCurveToMenu(PlotConfig &cfg)
{
    if (!m_curvesMenu || !cfg.series)
        return;

    QPixmap pixmap(16, 16);
    pixmap.fill(cfg.series->color());
    QIcon icon(pixmap);

    QMenu *curveMenu = m_curvesMenu->addMenu(icon, cfg.series->name());

    QAction *avgAction = curveMenu->addAction(tr("Average: N/A"));
    avgAction->setEnabled(false);
    cfg.avgAction = avgAction;

    QAction *stdevAction = curveMenu->addAction(tr("Stdev: N/A"));
    stdevAction->setEnabled(false);
    cfg.stdevAction = stdevAction;

    QAction *deleteAction = curveMenu->addAction(tr("Delete"));
    deleteAction->setToolTip(tr("Delete the curve"));
    deleteAction->setStatusTip(tr("Delete the curve"));
    QLineSeries *targetSeries = cfg.series;
    connect(deleteAction, &QAction::triggered, this, [this, targetSeries, curveMenu]() {
        removeCurve(targetSeries);
        delete curveMenu;
    });

    QAction *discreteAction = curveMenu->addAction(tr("Discrete"));
    discreteAction->setCheckable(true);
    discreteAction->setChecked(cfg.discrete);
    connect(discreteAction, &QAction::toggled, this, [this, targetSeries](bool checked) {
        std::scoped_lock lock(m_plotMutex);
        for (auto &plot : m_activePlots) {
            if (plot.series == targetSeries) {
                plot.discrete = checked;
                break;
            }
        }
    });
}

/**
 * @brief Obliterates a selected line trajectory correctly de-registering GUI and heap ties.
 * @param series Targets exactly which curve UI element triggered the destruction hook.
 */
void PlotterWindow::removeCurve(QLineSeries *series)
{
    if (!series)
        return;

    std::scoped_lock lock(m_plotMutex);

    for (int i = 0; i < m_activePlots.size(); ++i) {
        if (m_activePlots[i].series == series) {
            m_chart->removeSeries(series);
            m_activePlots[i].series = nullptr;
            m_activePlots.removeAt(i);
            delete series;
            series = nullptr;

            recalculateYBounds();

            QTimer::singleShot(10, m_legendManager, &ChartLegendManager::updateLegendPosition);
            m_statsNeedsRefresh = true;
            break;
        }
    }
}

/**
 * @brief Ensures overlay layout components reposition perfectly matching arbitrary desktop
 * reframes.
 */
void PlotterWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    m_legendManager->updateLegendPosition();
}

/**
 * @brief Interactively bolsters or weakens global plotting pixel strokes dynamically.
 */
void PlotterWindow::onLineThicknessChanged(int val)
{
    std::scoped_lock lock(m_plotMutex);
    for (auto &plot : m_activePlots) {
        if (!plot.series)
            continue;
        QPen p = plot.series->pen();
        p.setWidth(val);
        plot.series->setPen(p);
    }
}

/**
 * @brief Formal execution entry point instantiating process rules and UI execution contexts.
 */
/**
 * @brief Application entry point.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status code.
 */
int main(int argc, char *argv[])
{
    QCoreApplication::setApplicationVersion(QStringLiteral(PPRZ_VERSION_DESC));
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_plotter"));
    QCoreApplication::setApplicationName(QStringLiteral("Paparazzi Real-time Plotter"));

    QApplication app(argc, argv);
    QApplication::setStyle(new EditorLighteningStyle(QApplication::style()));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Paparazzi Real-time Plotter"));
    parser.addHelpOption();
    parser.addVersionOption();
    // Add same options as Logalizer/Plotter for the help display
    QCommandLineOption busOpt(QStringLiteral("b"),
                              QStringLiteral("ivy bus (Default is 127.255.255.255:2010)"),
                              QStringLiteral("ivy bus"));
    parser.addOption(busOpt);
    QCommandLineOption curveOpt(
            QStringLiteral("c"),
            QStringLiteral("Add a curve (e.g. '*:telemetry:BAT:voltage') or constant (e.g. '1.5'). "
                           "The curve is inserted into the last open window (cf -n option)"),
            QStringLiteral("curve"));
    parser.addOption(curveOpt);
    QCommandLineOption titleOpt(QStringLiteral("t"),
                                QStringLiteral("Set the last opened window title (cf -n option)"),
                                QStringLiteral("title"));
    parser.addOption(titleOpt);
    QCommandLineOption geomOpt(
            QStringLiteral("g"),
            QStringLiteral("Set the last opened window geometry ( '500x500+100+100' )"),
            QStringLiteral("geometry"));
    parser.addOption(geomOpt);
    QCommandLineOption newOpt(QStringLiteral("n"),
                              QStringLiteral("Open another window for the next curves"));
    parser.addOption(newOpt);
    QCommandLineOption memOpt(QStringLiteral("m"),
                              QStringLiteral("Memory size (default 500)"),
                              QStringLiteral("size"));
    parser.addOption(memOpt);
    QCommandLineOption updateOpt(QStringLiteral("u"),
                                 QStringLiteral("Update time in s (default 0.5)"),
                                 QStringLiteral("time"));
    parser.addOption(updateOpt);

    parser.process(app);

    // Initialize global config
    PlotterWindowConfig globalConfig;
    QString ivyBus = QStringLiteral("127.255.255.255:2010"); // default

    QList<PlotterWindowConfig> windowConfigs;
    PlotterWindowConfig currentConfig;

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

    for (int i = 1; i < mergedArgs.size(); ++i) {
        const QString &arg = mergedArgs[i];
        if (arg == QLatin1String("-b")) {
            if (i + 1 < mergedArgs.size()) {
                ivyBus = mergedArgs[++i];
            }
        } else if (arg == QLatin1String("-m")) {
            if (i + 1 < mergedArgs.size()) {
                globalConfig.memorySize = mergedArgs[++i].toInt();
            }
        } else if (arg == QLatin1String("-u")) {
            if (i + 1 < mergedArgs.size()) {
                globalConfig.updateTime = mergedArgs[++i].toDouble();
            }
        } else if (arg == QLatin1String("-n")) {
            windowConfigs.append(currentConfig);
            currentConfig = PlotterWindowConfig(); // start fresh
        } else if (arg == QLatin1String("-c")) {
            if (i + 1 < mergedArgs.size()) {
                currentConfig.curves.append(mergedArgs[++i]);
            }
        } else if (arg == QLatin1String("-t")) {
            if (i + 1 < mergedArgs.size()) {
                currentConfig.title = mergedArgs[++i];
            }
        } else if (arg == QLatin1String("-g")) {
            if (i + 1 < mergedArgs.size()) {
                currentConfig.geometry = mergedArgs[++i];
            }
        }
    }
    windowConfigs.append(currentConfig);

    // Setup Ivy Globally
    QString phome = qgetenv("PAPARAZZI_HOME");
    if (phome.isEmpty())
        phome = QStringLiteral("/home/%1/paparazzi").arg(qgetenv("USER"));
    QString xmlPath = phome + QStringLiteral("/var/messages.xml");

    pprzlink::MessageDictionary *g_dict = nullptr;
    pprzlink::IvyQtLink *g_link = nullptr;

    if (!QFile::exists(xmlPath)) {
        qWarning() << "Plotter: message dictionary not found at" << xmlPath
                   << ". Ivy telemetry will be disabled.";
    } else {
        try {
            g_dict = new pprzlink::MessageDictionary(xmlPath);
            g_link = new pprzlink::IvyQtLink(*g_dict, QStringLiteral("plotter"), &app);
            g_link->start(ivyBus);
        } catch (const std::exception &e) {
            qWarning() << "Plotter: failed to initialize Ivy link or message dictionary:"
                       << e.what();
            delete g_link;
            g_link = nullptr;
            delete g_dict;
            g_dict = nullptr;
        } catch (...) {
            qWarning() << "Plotter: unknown failure during Ivy initialization.";
            delete g_link;
            g_link = nullptr;
            delete g_dict;
            g_dict = nullptr;
        }
    }

    QString iconPath = QStringLiteral(":/penguin_icon_rtp.png");
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(QApplication::desktopFileName(),
                                   QStringLiteral("Paparazzi Real-Time Plotter"),
                                   QStringLiteral("Real-time plotter for telemetry messages"),
                                   iconPath,
                                   QStringLiteral("paparazzi-plotter"));
    QApplication::setWindowIcon(icon);

    QList<PlotterWindow *> windows;
    for (auto &cfg : windowConfigs) {
        if (globalConfig.memorySize != 500)
            cfg.memorySize = globalConfig.memorySize;
        // Float precision safe check
        if (std::abs(globalConfig.updateTime - 0.5) > 1e-5)
            cfg.updateTime = globalConfig.updateTime;

        PlotterWindow *w = new PlotterWindow(cfg, g_dict, g_link);
        w->setWindowIcon(icon);
        w->show();
        windows.append(w);
    }

    int ret = QApplication::exec();

    // Clean up
    if (g_link) {
        g_link->stop();
        delete g_link;
    }

    delete g_dict;

    return ret;
}

#include "plotter.moc"
