/**
 * @file logplotter.cpp
 * @brief Primary Entrypoint for the Paparazzi Log Plotter Toolkit.
 *
 * @details This file constructs the main UI context loop. It encompasses telemetry parsing,
 * dynamic data visualization scaling, and heavy multi-megabyte I/O log loading
 * optimized natively via Qt's C++ toolings.
 */
#include <algorithm>
#include <QAbstractSeries>
#include <QAction>
#include <QApplication>
#include <QByteArray>
#include <QChart>
#include <QChartView>
#include <QCheckBox>
#include <QComboBox>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QEvent>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
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
#include <QMap>
#include <QMargins>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMouseEvent>
#include <QObject>
#include <QPainter>
#include <QPair>
#include <QPen>
#include <QPixmap>
#include <QPointF>
#include <QProcess>
#include <QProgressDialog>
#include <QPushButton>
#include <QRectF>
#include <QRegularExpression>
#include <QResizeEvent>
#include <QSet>
#include <QShortcut>
#include <QSpinBox>
#include <QStandardPaths>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QValueAxis>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>
#include <QXmlStreamReader>
#include <utility>

#include "../include/os_desktop_utils.h"
#include "../include/pprz_version.h"
#include "plotter_common.h"

/**
 * @class ChartViewFilter
 * @brief Event filter routing explicit mouse and scroll controls dynamically into the Chart view.
 *
 * @details This decouples input logic (zooming with the scroll wheel, resetting bounds on
 * right-click) from subclassing native UI views, improving modularity.
 */
class ChartViewFilter : public QObject
{
    QChart *m_chart;
    std::function<void()> m_onZoom;

public:
    ChartViewFilter(QChart *chart, std::function<void()> onZoom, QObject *parent = nullptr)
        : QObject(parent)
        , m_chart(chart)
        , m_onZoom(std::move(onZoom))
    { }
    bool eventFilter(QObject *obj, QEvent *event) override
    {
        if (event->type() == QEvent::Wheel) {
            QWheelEvent *wheelEvent = static_cast<QWheelEvent *>(event);
            qreal factor = wheelEvent->angleDelta().y() > 0 ? 1.2 : 1.0 / 1.2;
            m_chart->zoom(factor);
            if (m_onZoom)
                m_onZoom();
            return true;
        } else if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
            if (mouseEvent->button() == Qt::RightButton) {
                m_chart->zoomReset();
                return true;
            }
        } else if (event->type() == QEvent::MouseButtonRelease) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                if (m_onZoom)
                    m_onZoom();
            }
        }
        return QObject::eventFilter(obj, event);
    }
};

/**
 * @class LogPlotterWindow
 * @brief The Core User Interface and execution context for rendering parsed telemetry logs.
 *
 * @details
 * Implements a heavyweight rendering application leveraging `QChart`. The window encompasses
 * both visual plotting (axes formatting, legend integrations) and heavy I/O abstractions
 * (parsing multi-megabyte log files asynchronously without blocking the UI thread).
 */
class LogPlotterWindow : public QMainWindow
{
    Q_OBJECT

public:
    LogPlotterWindow(QWidget *parent = nullptr)
        : QMainWindow(parent)
    {
        setAttribute(Qt::WA_DeleteOnClose);
        setWindowTitle(QStringLiteral("Log Plotter"));
        resize(900, 300);
        setupUI();
    }

private slots:
    /**
     * @brief Exports the current visual state of the chart to a rasterized image file.
     *
     * @details Prioritizes XDG-compliant `PicturesLocation` for automatic saving, intercepting
     * rendering engines to construct a high-resolution representation independent of native UI
     * scaling limits.
     */
    void saveScreenshot()
    {
        if (!m_chartView)
            return;

        QString picsLocation = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
        if (picsLocation.isEmpty()) {
            picsLocation = QDir::currentPath();
        }
        QString defaultPath = QDir(picsLocation).filePath(QStringLiteral("screenshot.png"));
        if (!m_currentLogFile.isEmpty()) {
            QFileInfo fi(m_currentLogFile);
            defaultPath = QDir(picsLocation)
                                  .filePath(QStringLiteral("pprz_log-") + fi.completeBaseName()
                                            + QStringLiteral(".png"));
        }

        QString fileName;
        {
            StderrBlocker blocker;
            QFileDialog dialog(this, tr("Save snapshot"), defaultPath);
            dialog.setAcceptMode(QFileDialog::AcceptSave);
            dialog.setNameFilters({tr("PNG Image (*.png)"),
                                   tr("JPEG Image (*.jpg)"),
                                   tr("WebP Image (*.webp)"),
                                   tr("BMP Image (*.bmp)")});
            dialog.setDefaultSuffix(QStringLiteral("png"));

            // Updates the default suffix whenever a new filter is selected from the combobox
            connect(&dialog,
                    &QFileDialog::filterSelected,
                    &dialog,
                    [&dialog](const QString &filter) {
                        if (filter.contains(QLatin1String("*.png")))
                            dialog.setDefaultSuffix(QStringLiteral("png"));
                        else if (filter.contains(QLatin1String("*.jpg")))
                            dialog.setDefaultSuffix(QStringLiteral("jpg"));
                        else if (filter.contains(QLatin1String("*.webp")))
                            dialog.setDefaultSuffix(QStringLiteral("webp"));
                        else if (filter.contains(QLatin1String("*.bmp")))
                            dialog.setDefaultSuffix(QStringLiteral("bmp"));
                    });

            if (dialog.exec() == QDialog::Accepted) {
                fileName = dialog.selectedFiles().first();
            }
        }

        if (!fileName.isEmpty()) {
            QPixmap pixmap(m_chartView->size() * 2);
            pixmap.fill(Qt::transparent);
            QPainter painter(&pixmap);
            painter.setRenderHint(QPainter::Antialiasing);
            painter.setRenderHint(QPainter::TextAntialiasing);
            m_chartView->scene()->render(&painter, QRectF(pixmap.rect()), m_chartView->sceneRect());
            painter.end();
            if (!pixmap.save(fileName)) {
                QMessageBox::warning(
                        this, tr("Error"), tr("Failed to save screenshot to %1").arg(fileName));
            } else {
                // qDebug() << "Screenshot saved to" << fileName;
            }
        }
    }

    void onAutoScaleToggled(bool checked)
    {
        m_edtMinY->setEnabled(!checked);
        m_edtMaxY->setEnabled(!checked);
        if (checked) {
            autoRescaleAxes();
        } else {
            onManualScaleChanged();
        }
    }

    void onManualScaleChanged()
    {
        if (!m_cbAutoScale->isChecked()) {
            bool okMin = false;
            bool okMax = false;
            double minY = m_edtMinY->text().toDouble(&okMin);
            double maxY = m_edtMaxY->text().toDouble(&okMax);
            if (okMin && okMax && minY < maxY) {
                m_axisY->setRange(minY, maxY);
            }
        }
    }

    void onAddConstantClicked()
    {
        QString text = m_edtConstant->text();
        QStringList parts = text.split(';');

        for (const QString &part : std::as_const(parts)) {
            bool ok = false;
            double val = part.trimmed().toDouble(&ok);
            if (!ok || !std::isfinite(val))
                continue;

            QLineSeries *series = new QLineSeries();
            series->setName(QStringLiteral("C=%1").arg(val));

            QPen pen = series->pen();
            pen.setColor(Qt::black);
            pen.setWidth(m_spnLineThickness->value());
            series->setPen(pen);

            double minX = 0;
            double maxX = 100;
            if (m_chart->series().count() > 0) {
                auto axes = m_chart->axes(Qt::Horizontal);
                if (!axes.isEmpty()) {
                    QValueAxis *axisX = qobject_cast<QValueAxis *>(axes.first());
                    if (axisX) {
                        minX = axisX->min();
                        maxX = axisX->max();
                    }
                } else {
                    auto *existing = m_chart->series().first();
                    QLineSeries *ls = qobject_cast<QLineSeries *>(existing);
                    if (ls && ls->count() > 0) {
                        minX = ls->at(0).x();
                        maxX = ls->at(ls->count() - 1).x();
                    }
                }
            }

            series->append(minX, val);
            series->append(maxX, val);

            m_chart->addSeries(series);
            series->attachAxis(m_axisX);
            series->attachAxis(m_axisY);
            autoRescaleAxes();

            if (m_curvesMenu) {
                QPixmap pixmap(16, 16);
                pixmap.fill(pen.color());
                QIcon icon(pixmap);
                QString title = QStringLiteral("C=%1").arg(val);
                QAction *deleteAction = m_curvesMenu->addAction(icon, title);
                deleteAction->setToolTip(tr("Delete constant curve"));
                deleteAction->setStatusTip(tr("Delete constant curve"));

                QLineSeries *targetSeries = series;
                connect(deleteAction,
                        &QAction::triggered,
                        this,
                        [this, targetSeries, deleteAction]() {
                            m_chart->removeSeries(targetSeries);
                            delete targetSeries;
                            deleteAction->deleteLater();
                            autoRescaleAxes();
                            if (m_legendManager)
                                m_legendManager->updateLegendPosition();
                            m_chartView->viewport()->update();
                        });
            }
        }

        QTimer::singleShot(15, this, [this]() {
            if (m_legendManager)
                m_legendManager->updateLegendPosition();
            m_chartView->viewport()->update();
        });
    }

    void exportFig()
    {
        QString docsLocation = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
        if (docsLocation.isEmpty())
            docsLocation = QDir::currentPath();
        QString defaultName = QDir(docsLocation)
                                      .filePath(QStringLiteral("pprz_log-")
                                                + QDateTime::currentDateTime().toString(
                                                        QStringLiteral("yy_MM_dd__HH_mm_ss"))
                                                + QStringLiteral(".fig"));
        QString fileName;
        {
            StderrBlocker blocker;
            fileName = QFileDialog::getSaveFileName(
                    this, tr("Export FIG"), defaultName, tr("FIG Files (*.fig)"));
        }
        if (fileName.isEmpty())
            return;

        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, tr("Error"), tr("Cannot write to file %1").arg(fileName));
            return;
        }

        QTextStream out(&file);
        out << "#FIG 3.2\n";
        out << "Landscape\n";
        out << "Center\n";
        out << "Metric\n";
        out << "A4\n";
        out << "100.00\n";
        out << "Single\n";
        out << "-2\n";
        out << "1200 2\n";

        double figWidth = 10000.0;
        double figHeight = 6000.0;
        double figOffsetX = 1000.0;
        double figOffsetY = 1000.0;

        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::lowest();

        QList<QAbstractSeries *> seriesList = m_chart->series();
        for (auto *series : std::as_const(seriesList)) {
            QLineSeries *lineSeries = qobject_cast<QLineSeries *>(series);
            if (lineSeries) {
                const auto &pts = lineSeries->points();
                for (const QPointF &pt : pts) {
                    minX = std::min(pt.x(), minX);
                    maxX = std::max(pt.x(), maxX);
                    minY = std::min(pt.y(), minY);
                    maxY = std::max(pt.y(), maxY);
                }
            }
        }

        if (minX >= maxX) {
            minX = 0;
            maxX = 1;
        }
        if (minY >= maxY) {
            minY = 0;
            maxY = 1;
        }

        double scaleX = figWidth / (maxX - minX);
        double scaleY = figHeight / (maxY - minY);

        // draw axes box
        out << "2 1 0 1 0 0 51 -1 -1 0.000 0 0 -1 0 0 5\n";
        out << "\t " << (int) figOffsetX << " " << (int) figOffsetY << " "
            << (int) (figOffsetX + figWidth) << " " << (int) figOffsetY << " "
            << (int) (figOffsetX + figWidth) << " " << (int) (figOffsetY + figHeight) << " "
            << (int) figOffsetX << " " << (int) (figOffsetY + figHeight) << " " << (int) figOffsetX
            << " " << (int) figOffsetY << "\n";

        int colorIndex = 1;
        for (auto *series : std::as_const(seriesList)) {
            QLineSeries *lineSeries = qobject_cast<QLineSeries *>(series);
            if (lineSeries && lineSeries->count() > 0) {
                int npoints = lineSeries->count();
                out << "2 1 0 1 " << (colorIndex % 32) << " 0 50 -1 -1 0.000 0 0 -1 0 0 " << npoints
                    << "\n\t";
                int count = 0;
                const auto &pts = lineSeries->points();
                for (const QPointF &pt : pts) {
                    int fx = static_cast<int>(figOffsetX + ((pt.x() - minX) * scaleX));
                    int fy = static_cast<int>(figOffsetY + figHeight - ((pt.y() - minY) * scaleY));
                    out << " " << fx << " " << fy;
                    if (++count >= 10) {
                        out << "\n\t";
                        count = 0;
                    }
                }
                out << "\n";
                colorIndex++;
            }
        }
    }

    void closeLogFile()
    {
        if (m_chart) {
            m_chart->removeAllSeries();
        }
        if (m_curvesMenu) {
            m_curvesMenu->clear();
        }
        for (QMenu *menu : std::as_const(m_logMenus)) {
            delete menu;
        }
        m_logMenus.clear();
        m_currentLogFile.clear();

        if (m_axisX)
            m_axisX->hide();
        if (m_axisY)
            m_axisY->hide();

        if (m_legendManager)
            m_legendManager->updateLegendPosition();
        if (m_chartView && m_chartView->viewport())
            m_chartView->viewport()->update();
    }

    /**
     * @brief Interactively prompts the user to select and load a raw `.log` telemetry file.
     *
     * @details Contains specific heuristic search rules. It attempts to traverse the
     * `PAPARAZZI_HOME` variables or up-folder structures strictly looking for the `var/logs`
     * environment map, falling back safely to the FHS standard `AppDataLocation` to guarantee users
     * aren't stranded in arbitrary paths.
     */
    void openLogFile()
    {
        QString fileName;
        {
            StderrBlocker blocker;
            const QString defaultLogExtPath = QStringLiteral("var/logs");
            QString logDir;
            QString envHome = qEnvironmentVariable("PAPARAZZI_HOME");
            if (envHome.isEmpty())
                envHome = qEnvironmentVariable("PAPARAZZI_SRC");
            if (!envHome.isEmpty()) {
                logDir = QDir(envHome).filePath(defaultLogExtPath);
            }
            if (logDir.isEmpty() || !QDir(logDir).exists()) {
                QDir searchDir(QCoreApplication::applicationDirPath());
                bool found = false;
                for (int i = 0; i < 4; ++i) {
                    if (QDir(searchDir.filePath(defaultLogExtPath)).exists()) {
                        logDir = searchDir.filePath(defaultLogExtPath);
                        found = true;
                        break;
                    }
                    if (!searchDir.cdUp())
                        break;
                }
                if (!found) {
                    if (QDir(QDir::current().filePath(defaultLogExtPath)).exists()) {
                        logDir = QDir::current().filePath(defaultLogExtPath);
                        found = true;
                    }
                }
                if (!found) {
                    QString dataLoc
                            = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                    logDir = QDir(dataLoc).filePath(QStringLiteral("logs"));
                }
            }
            fileName = QFileDialog::getOpenFileName(
                    this,
                    QStringLiteral("Open Paparazzi Log"),
                    logDir,
                    QStringLiteral("Log Files (*.log);;All Files (*)"));
        }
        if (!fileName.isEmpty()) {
            loadLogFile(fileName);
        }
    }

protected:
    void resizeEvent(QResizeEvent *event) override
    {
        QMainWindow::resizeEvent(event);
        if (m_legendManager) {
            m_legendManager->triggerRelayout();
        }
    }

private:
    QChart *m_chart = nullptr;
    QChartView *m_chartView = nullptr;
    QValueAxis *m_axisX = nullptr;
    QValueAxis *m_axisY = nullptr;
    QString m_currentLogFile;
    QString m_originallyLoadedFile;
    ChartLegendManager *m_legendManager = nullptr;
    QMap<QPair<QString, QString>, double> m_fieldCoefMap;
    QMenu *m_curvesMenu = nullptr;
    QCheckBox *m_cbAutoScale = nullptr;
    QLineEdit *m_edtMinY = nullptr;
    QLineEdit *m_edtMaxY = nullptr;
    QLineEdit *m_edtConstant = nullptr;
    QLineEdit *m_edtScaleNext = nullptr;
    QSpinBox *m_spnLineThickness = nullptr;
    QTimer *m_updateTimer = nullptr;
    QList<QMenu *> m_logMenus;

    void setupMenu()
    {
        QMenu *fileMenu = menuBar()->addMenu(tr("&File"));

        QAction *openAction = fileMenu->addAction(tr("Open Log"));
        openAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+O")));
        connect(openAction, &QAction::triggered, this, &LogPlotterWindow::openLogFile);

        QAction *newAction = fileMenu->addAction(tr("New"));
        newAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+N")));
        connect(newAction, &QAction::triggered, this, [this]() {
            QStringList args;
            if (!m_originallyLoadedFile.isEmpty()) {
                args << m_originallyLoadedFile;
            }

            // Start detached process for maximum robustness and memory isolation.
            // This ensures huge log files don't share identical process memory or block the current
            // UI thread.
            bool processStarted
                    = QProcess::startDetached(QCoreApplication::applicationFilePath(), args);

            if (!processStarted) {
                // Elegant fallback to in-process spawn if binary launching is unexpectedly
                // restricted.
                LogPlotterWindow *newWindow = new LogPlotterWindow();
                if (!m_originallyLoadedFile.isEmpty()) {
                    newWindow->loadLogFile(m_originallyLoadedFile);
                }
                newWindow->show();
            }
        });

        QAction *exportFigAction = fileMenu->addAction(tr("Export Fig"));
        exportFigAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+E")));
        connect(exportFigAction, &QAction::triggered, this, &LogPlotterWindow::exportFig);

        QAction *saveAction = fileMenu->addAction(tr("Save screenshot"));
        saveAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+S")));
        connect(saveAction, &QAction::triggered, this, &LogPlotterWindow::saveScreenshot);

        QAction *closeAction = fileMenu->addAction(tr("Close"));
        closeAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+W")));
        connect(closeAction, &QAction::triggered, this, &LogPlotterWindow::closeLogFile);

        fileMenu->addSeparator();

        QAction *quitAction = fileMenu->addAction(tr("Quit"));
        quitAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+Q")));
        connect(quitAction, &QAction::triggered, qApp, &QApplication::quit);

        m_curvesMenu = menuBar()->addMenu(tr("&Curves"));
        m_curvesMenu->setToolTipsVisible(true);
    }

    void setupUI()
    {
        setupMenu();

        QWidget *mainWidget = new QWidget(this);
        setCentralWidget(mainWidget);
        QVBoxLayout *mainLayout = new QVBoxLayout(mainWidget);
        mainLayout->setContentsMargins(0, 0, 0, 0);
        mainLayout->setSpacing(0);

        QWidget *toolbarWidget = new QWidget();
        QHBoxLayout *toolbarLayout = new QHBoxLayout(toolbarWidget);
        toolbarLayout->setContentsMargins(2, 2, 2, 2);

        m_cbAutoScale = new QCheckBox(QStringLiteral("Auto Scale"));
        m_cbAutoScale->setChecked(true);

        m_edtMinY = new QLineEdit();
        m_edtMaxY = new QLineEdit();
        m_edtMinY->setMaximumWidth(90);
        m_edtMaxY->setMaximumWidth(90);
        m_edtMinY->setEnabled(false);
        m_edtMaxY->setEnabled(false);

        QLabel *lblConst = new QLabel(QStringLiteral("Constant"));
        m_edtConstant = new QLineEdit();
        m_edtConstant->setMaximumWidth(75);

        QLabel *lblScaleNext = new QLabel(QStringLiteral("Scale next by"));
        m_edtScaleNext = new QLineEdit(QStringLiteral("1.0+0"));
        m_edtScaleNext->setToolTip(
                tr("Format: scale+transpose or scale-transpose (e.g. 2.0+10.5)"));
        m_edtScaleNext->setMaximumWidth(75);

        m_spnLineThickness = new QSpinBox();
        m_spnLineThickness->setToolTip(QStringLiteral("Line Thickness (px)"));
        m_spnLineThickness->setRange(1, 10);
        m_spnLineThickness->setValue(1);
        m_spnLineThickness->hide();

        m_updateTimer = new QTimer(this);

        toolbarLayout->addWidget(m_cbAutoScale);
        connect(m_cbAutoScale, &QCheckBox::toggled, this, &LogPlotterWindow::onAutoScaleToggled);
        connect(m_edtMinY,
                &QLineEdit::editingFinished,
                this,
                &LogPlotterWindow::onManualScaleChanged);
        connect(m_edtMaxY,
                &QLineEdit::editingFinished,
                this,
                &LogPlotterWindow::onManualScaleChanged);
        toolbarLayout->addWidget(new QLabel(QStringLiteral("Min")));
        toolbarLayout->addWidget(m_edtMinY);
        toolbarLayout->addWidget(new QLabel(QStringLiteral("Max")));
        toolbarLayout->addWidget(m_edtMaxY);

        toolbarLayout->addWidget(lblConst);
        toolbarLayout->addWidget(m_edtConstant);
        connect(m_edtConstant,
                &QLineEdit::editingFinished,
                this,
                &LogPlotterWindow::onAddConstantClicked);
        toolbarLayout->addWidget(lblScaleNext);
        toolbarLayout->addWidget(m_edtScaleNext);
        QLabel *lblLineThickness = new QLabel(QStringLiteral("Line:"));
        lblLineThickness->hide();
        toolbarLayout->addWidget(lblLineThickness);
        toolbarLayout->addWidget(m_spnLineThickness);
        toolbarLayout->addStretch();

        m_chartView = new QChartView();
        m_chartView->setContentsMargins(0, 0, 0, 0);
        m_chartView->setFrameShape(QFrame::NoFrame);
        m_chartView->setRubberBand(QChartView::RectangleRubberBand);

        // Zooming and Panning Shortcuts
        QShortcut *zoomInSc = new QShortcut(QKeySequence(Qt::Key_Plus), this);
        connect(zoomInSc, &QShortcut::activated, this, [this]() {
            if (m_chart) {
                m_chart->zoomIn();
                m_cbAutoScale->setChecked(false);
            }
        });
        QShortcut *zoomOutSc = new QShortcut(QKeySequence(Qt::Key_Minus), this);
        connect(zoomOutSc, &QShortcut::activated, this, [this]() {
            if (m_chart) {
                m_chart->zoomOut();
                m_cbAutoScale->setChecked(false);
            }
        });
        QShortcut *zoomResetSc = new QShortcut(QKeySequence(Qt::Key_0), this);
        connect(zoomResetSc, &QShortcut::activated, this, [this]() {
            if (m_chart)
                m_chart->zoomReset();
        });

        QShortcut *panLeftSc = new QShortcut(QKeySequence(Qt::Key_Left), this);
        connect(panLeftSc, &QShortcut::activated, this, [this]() {
            if (m_chart) {
                m_chart->scroll(-50, 0);
                m_cbAutoScale->setChecked(false);
            }
        });
        QShortcut *panRightSc = new QShortcut(QKeySequence(Qt::Key_Right), this);
        connect(panRightSc, &QShortcut::activated, this, [this]() {
            if (m_chart) {
                m_chart->scroll(50, 0);
                m_cbAutoScale->setChecked(false);
            }
        });
        QShortcut *panUpSc = new QShortcut(QKeySequence(Qt::Key_Up), this);
        connect(panUpSc, &QShortcut::activated, this, [this]() {
            if (m_chart) {
                m_chart->scroll(0, 50);
                m_cbAutoScale->setChecked(false);
            }
        });
        QShortcut *panDownSc = new QShortcut(QKeySequence(Qt::Key_Down), this);
        connect(panDownSc, &QShortcut::activated, this, [this]() {
            if (m_chart) {
                m_chart->scroll(0, -50);
                m_cbAutoScale->setChecked(false);
            }
        });

        m_chart = new QChart();
        m_chart->setMargins(QMargins(0, 0, 0, 0));
        m_chart->layout()->setContentsMargins(0, 0, 0, 0);
        m_chart->setBackgroundRoundness(0);
        m_chart->setBackgroundPen(QPen(Qt::NoPen));
        m_chartView->setChart(m_chart);
        m_chartView->setRenderHint(QPainter::Antialiasing);
        m_chartView->viewport()->installEventFilter(new ChartViewFilter(
                m_chart, [this]() { m_cbAutoScale->setChecked(false); }, m_chartView));

        m_axisX = new QValueAxis();
        m_axisX->setLabelFormat(QStringLiteral("%gs"));
        m_axisY = new QValueAxis();

        m_axisX->hide();
        m_axisY->hide();

        m_chart->addAxis(m_axisX, Qt::AlignBottom);
        m_chart->addAxis(m_axisY, Qt::AlignLeft);

        mainLayout->addWidget(toolbarWidget);
        mainLayout->addWidget(m_chartView);

        m_legendManager = new ChartLegendManager(m_chart, m_chartView);
    }

    void autoRescaleAxes()
    {
        double calcMinX = std::numeric_limits<double>::infinity();
        double calcMaxX = -std::numeric_limits<double>::infinity();
        double calcMinY = std::numeric_limits<double>::infinity();
        double calcMaxY = -std::numeric_limits<double>::infinity();
        bool hasData = false;

        const auto &seriesList = m_chart->series();
        for (auto *s : seriesList) {
            QLineSeries *ls = qobject_cast<QLineSeries *>(s);
            if (ls && ls->count() > 0) {
                hasData = true;
                const auto &pts = ls->points();
                for (const QPointF &p : pts) {
                    calcMinX = std::min(p.x(), calcMinX);
                    calcMaxX = std::max(p.x(), calcMaxX);
                    calcMinY = std::min(p.y(), calcMinY);
                    calcMaxY = std::max(p.y(), calcMaxY);
                }
            }
        }
        if (hasData) {
            if (calcMinX == calcMaxX) {
                calcMinX -= 1;
                calcMaxX += 1;
            }
            if (calcMinY == calcMaxY) {
                calcMinY -= 1;
                calcMaxY += 1;
            }
            double marginY = (calcMaxY - calcMinY) * 0.05;

            m_axisX->setRange(calcMinX, calcMaxX);

            if (m_cbAutoScale->isChecked()) {
                m_axisY->setRange(calcMinY - marginY, calcMaxY + marginY);
                m_edtMinY->setText(QString::number(calcMinY - marginY, 'f', 2));
                m_edtMaxY->setText(QString::number(calcMaxY + marginY, 'f', 2));
            } else {
                bool okMin = false;
                bool okMax = false;
                double minY = m_edtMinY->text().toDouble(&okMin);
                double maxY = m_edtMaxY->text().toDouble(&okMax);
                if (okMin && okMax && minY < maxY) {
                    m_axisY->setRange(minY, maxY);
                }
            }

            m_axisX->show();
            m_axisY->show();
        } else {
            m_axisX->hide();
            m_axisY->hide();
        }
    }

    void
    addCurve(const QString &acId, const QString &msgName, const QString &fieldName, int fieldIndex)
    {
        if (m_currentLogFile.isEmpty())
            return;
        QFile file(m_currentLogFile);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
            return;

        QString txt = m_edtScaleNext->text().trimmed();
        txt.replace(QLatin1String(" "), QLatin1String(""));
        double scale = 1.0;
        double transpose = 0.0;

        int splitIdx = -1;
        for (int i = 1; i < txt.length(); ++i) {
            if (txt.at(i) == '+' || txt.at(i) == '-') {
                if (txt.at(i - 1) != 'e' && txt.at(i - 1) != 'E') {
                    splitIdx = i;
                    break;
                }
            }
        }

        if (splitIdx != -1) {
            bool ok1 = false;
            bool ok2 = false;
            double s = QStringView(txt).left(splitIdx).toDouble(&ok1);
            if (ok1)
                scale = s;
            double t = QStringView(txt).mid(splitIdx).toDouble(&ok2);
            if (ok2)
                transpose = t;
        } else {
            bool ok = false;
            double s = txt.toDouble(&ok);
            if (ok)
                scale = s;
        }

        QPair<QString, QString> dictKey = qMakePair(msgName, fieldName);
        if (m_fieldCoefMap.contains(dictKey)) {
            scale *= m_fieldCoefMap[dictKey];
        }

        QFileInfo fi(m_currentLogFile);
        QString logName = fi.baseName();
        QString logNameCondensed = logName;
        logNameCondensed.replace(QLatin1String("__"), QLatin1String("#TEMP#"));
        logNameCondensed.replace(QLatin1String("_"), QLatin1String(""));
        logNameCondensed.replace(QLatin1String("#TEMP#"), QLatin1String("_"));

        QString curveTitle = logName + QStringLiteral(":") + acId + QStringLiteral(":") + msgName
                + QStringLiteral(":") + fieldName + QStringLiteral(":") + QString::number(scale)
                + QStringLiteral("+") + QString::number(transpose);
        QString curveTitleCondensed = logNameCondensed + QStringLiteral(":") + acId
                + QStringLiteral(":") + msgName + QStringLiteral(":") + fieldName
                + QStringLiteral(":") + QString::number(scale) + QStringLiteral("+")
                + QString::number(transpose);

        QLineSeries *series = new QLineSeries();
        series->setName(curveTitle);

        file.close(); // Not using the standard QTextStream any more

        processLogLines(m_currentLogFile,
                        acId,
                        nullptr,
                        [&]([[maybe_unused]] const QString &timeStr,
                            const QString &msgNameStr,
                            const QString &dataPart,
                            const QRegularExpression &spaceRe) {
                            if (msgNameStr == msgName) {
                                QStringList values;
                                if (!dataPart.isEmpty()) {
                                    values = dataPart.split(spaceRe, Qt::SkipEmptyParts);
                                }
                                if (values.size() > fieldIndex) {
                                    bool okTime;
                                    bool okVal;
                                    double t = timeStr.toDouble(&okTime);
                                    double v = values[fieldIndex].toDouble(&okVal);
                                    if (okTime && okVal) {
                                        series->append(t, (v * scale) + transpose);
                                    }
                                }
                            }
                        });

        if (series->count() > 0) {
            QPen pen = series->pen();
            pen.setColor(getNextSaturatedColor());
            pen.setWidth(m_spnLineThickness->value());
            series->setPen(pen);

            m_chart->addSeries(series);
            series->attachAxis(m_axisX);
            series->attachAxis(m_axisY);
            autoRescaleAxes();

            if (m_curvesMenu) {
                QPixmap pixmap(16, 16);
                pixmap.fill(pen.color());
                QIcon icon(pixmap);
                QAction *deleteAction = m_curvesMenu->addAction(icon, curveTitleCondensed);
                deleteAction->setToolTip(tr("Delete curve"));
                deleteAction->setStatusTip(tr("Delete curve"));

                QLineSeries *targetSeries = series;
                connect(deleteAction,
                        &QAction::triggered,
                        this,
                        [this, targetSeries, deleteAction]() {
                            m_chart->removeSeries(targetSeries);
                            delete targetSeries;
                            deleteAction->deleteLater();
                            autoRescaleAxes();
                            if (m_legendManager)
                                m_legendManager->updateLegendPosition();
                            m_chartView->viewport()->update();
                        });
            }

            QTimer::singleShot(15, this, [this]() {
                if (m_legendManager)
                    m_legendManager->updateLegendPosition();
                m_chartView->viewport()->update();
            });
        } else {
            delete series;
        }
    }

public:
    /**
     * @brief Asynchronous, zero-allocation log chunk parser.
     * @tparam Func Lambda callback signature for handling regex matching per sequence line.
     * @param logFileName The absolute path to the data file being processed.
     * @param acId The target Aircraft ID to selectively parse.
     * @param progress Optional pointer to UI progress tracking.
     * @param callback Executor firing upon every isolated data block.
     *
     * @details
     * THE BIG GOTCHA (Performance Strategy):
     * Native iteration strings (e.g. `QTextStream::readLine()`) are devastatingly slow for files >
     * 10MB because they dynamically allocate heap memory for every single generated string buffer.
     *
     * THE "WHY":
     * This function reads flat blocks of 1,048,576 bytes (1MB chunks) straight into memory,
     * stepping via pure `char*` pointers. Memory extraction into actual readable `QString`
     * constructs is ONLY permitted functionally if the target Aircraft ID cleanly matches the
     * strict sequence check, evading thousands of useless allocations per tick.
     */
    template <typename Func>
    void processLogLines(const QString &logFileName,
                         const QString &acId,
                         QProgressDialog *progress,
                         Func callback)
    {
        QFile file(logFileName);
        if (!file.open(QIODevice::ReadOnly))
            return;

        QByteArray acIdBytes = acId.toUtf8();
        const char *targetAcId = acIdBytes.constData();
        int targetAcIdLen = acIdBytes.length();

        const int CHUNK_SIZE = 1048576;
        QByteArray buffer;
        QRegularExpression spaceRe(QStringLiteral("\\s+"));

        while (!file.atEnd()) {
            buffer.append(file.read(CHUNK_SIZE));
            if (progress) {
                if (progress->wasCanceled())
                    break;
                progress->setValue(file.pos());
                QCoreApplication::processEvents();
            }

            int lineStart = 0;
            while (true) {
                int nlIdx = buffer.indexOf('\n', lineStart);
                if (nlIdx == -1)
                    break;

                int lineLen = nlIdx - lineStart;
                if (lineLen > 0 && buffer.at(nlIdx - 1) == '\r')
                    lineLen--;

                if (lineLen > 0) {
                    const char *lineData = buffer.constData() + lineStart;
                    int s1 = -1;
                    int len1 = 0;
                    int s2 = -1;
                    int len2 = 0;
                    int s3 = -1;
                    int len3 = 0;
                    int dataStart = -1;

                    for (int i = 0; i < lineLen; ++i) {
                        if (lineData[i] != ' ' && lineData[i] != '\t' && lineData[i] != '\r') {
                            if (s1 == -1) {
                                s1 = i;
                            } else if (len1 > 0 && s2 == -1) {
                                s2 = i;
                            } else if (len2 > 0 && s3 == -1) {
                                s3 = i;
                            } else if (len3 > 0 && dataStart == -1) {
                                dataStart = i;
                                break;
                            }
                        } else {
                            if (s1 != -1 && s2 == -1) {
                                len1 = i - s1;
                            } else if (s2 != -1 && s3 == -1) {
                                len2 = i - s2;
                            } else if (s3 != -1 && len3 == 0) {
                                len3 = i - s3;
                            }
                        }
                    }
                    if (s3 != -1 && len3 == 0)
                        len3 = lineLen - s3;

                    if (s2 != -1 && len2 > 0 && s3 != -1 && len3 > 0) {
                        if (len2 == targetAcIdLen
                            && qstrncmp(lineData + s2, targetAcId, len2) == 0) {
                            QString timeStr = QString::fromUtf8(lineData + s1, len1);
                            QString msgNameStr = QString::fromUtf8(lineData + s3, len3);

                            QString dataPart;
                            if (dataStart != -1) {
                                dataPart = QString::fromUtf8(lineData + dataStart,
                                                             lineLen - dataStart);
                            }

                            callback(timeStr, msgNameStr, dataPart, spaceRe);
                        }
                    }
                }
                lineStart = nlIdx + 1;
            }
            buffer.remove(0, lineStart);
        }
    }

    void doExportCsv(const QString &acId,
                     const QString &inFileStr,
                     const QString &outFileName,
                     const QMap<QString, QList<int>> &selectedFields,
                     const QMap<QString, QStringList> &selectedFieldNames,
                     const QStringList &headerCols,
                     const QString &delimiter,
                     QProgressDialog *progress = nullptr)
    {
        QFile outFile(outFileName);
        if (!outFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this,
                                 QStringLiteral("Export CSV"),
                                 QStringLiteral("Cannot write to CSV file."));
            return;
        }

        QTextStream out(&outFile);
        out << headerCols.join(delimiter) << "\n";

        processLogLines(inFileStr,
                        acId,
                        progress,
                        [&]([[maybe_unused]] const QString &timeStr,
                            const QString &msgName,
                            const QString &dataPart,
                            const QRegularExpression &spaceRe) {
                            if (selectedFields.contains(msgName)) {
                                QStringList values;
                                if (!dataPart.isEmpty()) {
                                    values = dataPart.split(spaceRe, Qt::SkipEmptyParts);
                                }

                                QStringList row;
                                row << timeStr;

                                for (int i = 1; i < headerCols.size(); ++i) {
                                    const QString &col = headerCols[i];
                                    if (col.startsWith(msgName + QStringLiteral("."))) {
                                        int fIdx = selectedFieldNames[msgName].indexOf(col);
                                        if (fIdx != -1 && fIdx < selectedFields[msgName].size()) {
                                            int paramIdx = selectedFields[msgName][fIdx];
                                            if (paramIdx < values.size()) {
                                                row << values[paramIdx].trimmed();
                                            } else {
                                                row << QLatin1String("");
                                            }
                                        } else {
                                            row << QLatin1String("");
                                        }
                                    } else {
                                        row << QLatin1String("");
                                    }
                                }
                                out << row.join(delimiter) << "\n";
                            }
                        });

        if (progress)
            progress->setValue(progress->maximum());
    }

    void loadLogFile(const QString &fileName, bool autoExportCsv = false)
    {
        QApplication::setOverrideCursor(Qt::WaitCursor);

        m_originallyLoadedFile = fileName;
        QString dataFileName = fileName;
        QMap<QString, QStringList> dictFields;

        if (fileName.endsWith(QLatin1String(".log"), Qt::CaseInsensitive)) {
            QFile logFile(fileName);
            if (logFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
                QString content = logFile.readAll();

                QRegularExpression reDataFile(
                        QStringLiteral("<configuration[^>]*data_file=\"([^\"]+)\""));
                QRegularExpressionMatch matchDataFile = reDataFile.match(content);
                if (matchDataFile.hasMatch()) {
                    QString dFile = matchDataFile.captured(1);
                    QFileInfo fi(fileName);
                    dataFileName = fi.absoluteDir().filePath(dFile);
                }

                int protoStart = content.indexOf(QLatin1String("<protocol>"));
                int protoEnd = content.indexOf(QLatin1String("</protocol>"), protoStart);
                if (protoStart != -1 && protoEnd != -1) {
                    // printf("Found protocol from %d to %d\n", protoStart, protoEnd);
                    // fflush(stdout);//Enable for Debug only
                    QString protocolXml = content.mid(protoStart, protoEnd - protoStart + 11);
                    QXmlStreamReader xml(protocolXml);
                    while (!xml.atEnd() && !xml.hasError()) {
                        QXmlStreamReader::TokenType token = xml.readNext();
                        if (token == QXmlStreamReader::StartElement) {
                            if (xml.name().toString() == QLatin1String("message")
                                && xml.attributes().hasAttribute("NAME")) {
                                QString msgName = xml.attributes().value("NAME").toString();
                                QStringList fields;
                                while ((xml.tokenType() != QXmlStreamReader::EndElement
                                        || xml.name().toString() != QLatin1String("message"))
                                       && !xml.atEnd()) {
                                    xml.readNext();
                                    if (xml.tokenType() == QXmlStreamReader::StartElement
                                        && xml.name().toString() == QLatin1String("field")) {
                                        if (xml.attributes().hasAttribute("NAME")) {
                                            QString fName
                                                    = xml.attributes().value("NAME").toString();
                                            fields.append(fName);
                                            if (xml.attributes().hasAttribute("ALT_UNIT_COEF")) {
                                                bool ok = false;
                                                double coef = xml.attributes()
                                                                      .value("ALT_UNIT_COEF")
                                                                      .toDouble(&ok);
                                                if (ok) {
                                                    m_fieldCoefMap[qMakePair(msgName, fName)]
                                                            = coef;
                                                }
                                            }
                                        }
                                    }
                                }
                                dictFields[msgName] = fields;
                                // printf("Extracted msg: %s with %lld fields\n",
                                // msgName.toStdString().c_str(), fields.size());
                                // fflush(stdout);//Enable for Debug only
                            }
                        }
                    }
                }
            }
        }

        if (!dataFileName.isEmpty() && m_currentLogFile == dataFileName) {
            while (QApplication::overrideCursor())
                QApplication::restoreOverrideCursor();
            return;
        }

        m_currentLogFile = dataFileName;
        QFile file(dataFileName);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            while (QApplication::overrideCursor())
                QApplication::restoreOverrideCursor();
            QMessageBox::warning(this,
                                 QStringLiteral("Error"),
                                 QStringLiteral("Cannot open file ") + dataFileName);
            return;
        }

        qint64 totalSize = file.size();
        QProgressDialog progress(
                tr("Parsing log file..."), tr("Cancel"), 0, totalSize > 0 ? totalSize : 1, this);
        progress.setWindowModality(Qt::WindowModal);
        progress.setMinimumDuration(200);

        QSet<QPair<QString, QString>> acMsgPairs;

        // Fast parsing of the data file
        const int CHUNK_SIZE
                = 1024 * 1024; // Adjust chunk size as you deem fit for performance/memory balance
        QByteArray buffer;
        while (!file.atEnd()) {
            if (progress.wasCanceled()) {
                m_currentLogFile.clear();
                while (QApplication::overrideCursor())
                    QApplication::restoreOverrideCursor();
                return;
            }
            buffer.append(file.read(CHUNK_SIZE));
            progress.setValue(file.pos());
            QCoreApplication::processEvents();

            int lineStart = 0;
            int nlIdx = 0;

            while ((nlIdx = buffer.indexOf('\n', lineStart)) != -1) {
                int lineLen = nlIdx - lineStart;
                if (lineLen > 0) {
                    const char *lineData = buffer.constData() + lineStart;

                    int s1 = -1;
                    int len1 = 0;
                    int s2 = -1;
                    int len2 = 0;
                    int s3 = -1;
                    int len3 = 0;

                    for (int i = 0; i < lineLen; ++i) {
                        if (lineData[i] != ' ' && lineData[i] != '\t' && lineData[i] != '\r') {
                            if (s1 == -1) {
                                s1 = i;
                            } else if (len1 > 0 && s2 == -1) {
                                s2 = i;
                            } else if (len2 > 0 && s3 == -1) {
                                s3 = i;
                            }
                        } else {
                            if (s1 != -1 && s2 == -1) {
                                len1 = i - s1;
                            } else if (s2 != -1 && s3 == -1) {
                                len2 = i - s2;
                            } else if (s3 != -1 && len3 == 0) {
                                len3 = i - s3;
                                break;
                            }
                        }
                    }
                    if (s3 != -1 && len3 == 0) {
                        len3 = lineLen - s3;
                    }

                    if (s2 != -1 && len2 > 0 && s3 != -1 && len3 > 0) {
                        QString acId = QString::fromUtf8(lineData + s2, len2);
                        QString msgName = QString::fromUtf8(lineData + s3, len3);
                        acMsgPairs.insert(qMakePair(acId, msgName));
                    }
                }
                lineStart = nlIdx + 1;
            }
            buffer.remove(0, lineStart);
        }

        QFileInfo fi(fileName);
        QString logName = fi.baseName();

        QMap<QString, QSet<QString>> acToMsgs;
        for (const auto &pair : acMsgPairs) {
            acToMsgs[pair.first].insert(pair.second);
        }

        for (auto it = acToMsgs.begin(); it != acToMsgs.end(); ++it) {
            const QString &acId = it.key();

            QString menuTitle = logName + QStringLiteral(":") + acId;
            QMenu *acMenu = menuBar()->addMenu(menuTitle);
            m_logMenus.append(acMenu);

            QStringList msgs = it.value().values();
            msgs.sort(); // Sorting messages alphabetically

            for (const QString &msgName : std::as_const(msgs)) {
                if (dictFields.contains(msgName)) {
                    // printf("Found MSG in dict: %s\n", msgName.toStdString().c_str());
                    // fflush(stdout);//Enable for Debug only
                    QMenu *msgMenu = acMenu->addMenu(msgName);
                    const QStringList &fields = dictFields.value(msgName);
                    for (int i = 0; i < fields.size(); ++i) {
                        QString fieldName = fields.at(i);
                        QAction *fieldAction = msgMenu->addAction(fieldName);
                        connect(fieldAction,
                                &QAction::triggered,
                                this,
                                [this, acId, msgName, fieldName, i]() {
                                    this->addCurve(acId, msgName, fieldName, i);
                                });
                    }
                } else {
                    acMenu->addAction(msgName);
                }
            }

            acMenu->addSeparator();

            QAction *exportKmlAction = acMenu->addAction(QStringLiteral("Export KML"));
            connect(exportKmlAction,
                    &QAction::triggered,
                    this,
                    [this, acId, logName, dictFields]() {
                        QString docsLocation = QStandardPaths::writableLocation(
                                QStandardPaths::DocumentsLocation);
                        if (docsLocation.isEmpty())
                            docsLocation = QFileInfo(m_currentLogFile).path();
                        QString defaultName
                                = QDir(docsLocation)
                                          .filePath(
                                                  logName + QStringLiteral(":") + acId
                                                  + QStringLiteral(
                                                          ".kml")); // Well maybe a - is better than
                                                                    // : for Windowsfile systems,
                                                                    // but we keep it consistent
                                                                    // with the previous OCAML code

                        QString fileName;
                        {
                            StderrBlocker blocker;
                            fileName = QFileDialog::getSaveFileName(
                                    this, tr("Export KML"), defaultName, tr("KML Files (*.kml)"));
                        }
                        if (fileName.isEmpty())
                            return;

                        QFile file(m_currentLogFile);
                        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
                            QMessageBox::warning(this,
                                                 QStringLiteral("Export KML"),
                                                 QStringLiteral("Cannot open data file."));
                            return;
                        }

                        int latIdx = -1;
                        int lonIdx = -1;
                        int altIdx = -1;
                        int utmEastIdx = -1;
                        int utmNorthIdx = -1;
                        int utmZoneIdx = -1;
                        QString targetMsg;
                        bool isUtm = false;
                        double latScale = 1.0;
                        double lonScale = 1.0;
                        double altScale = 1.0;

                        if (dictFields.contains(QStringLiteral("GPS"))) {
                            targetMsg = QStringLiteral("GPS");
                            utmEastIdx = dictFields[QStringLiteral("GPS")].indexOf("utm_east");
                            utmNorthIdx = dictFields[QStringLiteral("GPS")].indexOf("utm_north");
                            utmZoneIdx = dictFields[QStringLiteral("GPS")].indexOf("utm_zone");
                            altIdx = dictFields[QStringLiteral("GPS")].indexOf("alt");
                            altScale = 1e-3;
                            isUtm = true;
                        } else if (dictFields.contains(QStringLiteral("GPS_INT"))) {
                            targetMsg = QStringLiteral("GPS_INT");
                            latIdx = dictFields[QStringLiteral("GPS_INT")].indexOf("lat");
                            lonIdx = dictFields[QStringLiteral("GPS_INT")].indexOf("lon");
                            altIdx = dictFields[QStringLiteral("GPS_INT")].indexOf("hmsl");
                            if (altIdx == -1)
                                altIdx = dictFields[QStringLiteral("GPS_INT")].indexOf("alt");
                            latScale = 1e-7;
                            lonScale = 1e-7;
                            altScale = 1e-3;
                        } else if (dictFields.contains(QStringLiteral("MINIMAL_COM"))) {
                            targetMsg = QStringLiteral("MINIMAL_COM");
                            latIdx = dictFields[QStringLiteral("MINIMAL_COM")].indexOf("lat");
                            lonIdx = dictFields[QStringLiteral("MINIMAL_COM")].indexOf("lon");
                            altIdx = dictFields[QStringLiteral("MINIMAL_COM")].indexOf("hmsl");
                            if (altIdx == -1)
                                altIdx = dictFields[QStringLiteral("MINIMAL_COM")].indexOf("alt");
                        } else {
                            // Try to dynamically figure it out from file scanning if not correctly
                            // in dict
                            QFile checkFile(m_currentLogFile);
                            if (checkFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
                                QTextStream checkIn(&checkFile);
                                QRegularExpression checkRe(QStringLiteral("\\s+"));
                                while (!checkIn.atEnd()) {
                                    QString line = checkIn.readLine();
                                    QStringList parts = line.split(checkRe, Qt::SkipEmptyParts);
                                    if (parts.size() > 3 && parts[1] == acId) {
                                        if (parts[2] == QLatin1String("GPS")) {
                                            targetMsg = QStringLiteral("GPS");
                                            break;
                                        } else if (parts[2] == QLatin1String("GPS_INT")) {
                                            targetMsg = QStringLiteral("GPS_INT");
                                            break;
                                        } else if (parts[2] == QLatin1String("MINIMAL_COM")) {
                                            targetMsg = QStringLiteral("MINIMAL_COM");
                                            break;
                                        }
                                    }
                                }
                            }
                            if (targetMsg == QLatin1String("GPS")
                                && dictFields.contains(QStringLiteral("GPS"))) {
                                utmEastIdx = dictFields[QStringLiteral("GPS")].indexOf("utm_east");
                                utmNorthIdx
                                        = dictFields[QStringLiteral("GPS")].indexOf("utm_north");
                                utmZoneIdx = dictFields[QStringLiteral("GPS")].indexOf("utm_zone");
                                altIdx = dictFields[QStringLiteral("GPS")].indexOf("alt");
                                altScale = 1e-3;
                                isUtm = true;
                            }
                        }

                        if (targetMsg.isEmpty() || (!isUtm && (latIdx == -1 || lonIdx == -1))
                            || (isUtm
                                && (utmEastIdx == -1 || utmNorthIdx == -1 || utmZoneIdx == -1))) {
                            QMessageBox::warning(
                                    this,
                                    QStringLiteral("Export KML"),
                                    QStringLiteral("Could not find valid GPS coordinates in the "
                                                   "log for this AC."));
                            return;
                        }

                        file.close(); // Abandon standard file iter for chunk processor
                        QString kmlCoords;

                        auto utm2deg = [](double x, double y, int zone, double &lat, double &lon) {
                            double a = 6378137.0;
                            double eccSquared = 0.00669438000426224;
                            double k0 = 0.9996;
                            double eccPrimeSquared = eccSquared / (1.0 - eccSquared);
                            x = x / (k0 * a);
                            y = y / k0;
                            double m = y / a;
                            double mu = m
                                    / (1.0 - eccSquared / 4.0 - 3.0 * eccSquared * eccSquared / 64.0
                                       - 5.0 * pow(eccSquared, 3) / 256.0);
                            double e1 = (1.0 - sqrt(1.0 - eccSquared))
                                    / (1.0 + sqrt(1.0 - eccSquared));

                            double phi1Rad = mu
                                    + ((3.0 * e1 / 2.0 - 27.0 * pow(e1, 3) / 32.0) * sin(2.0 * mu))
                                    + ((21.0 * e1 * e1 / 16.0 - 55.0 * pow(e1, 4) / 32.0)
                                       * sin(4.0 * mu))
                                    + ((151.0 * pow(e1, 3) / 96.0) * sin(6.0 * mu));
                            double N1 = a / sqrt(1.0 - (eccSquared * pow(sin(phi1Rad), 2)));
                            double T1 = pow(tan(phi1Rad), 2);
                            double C1 = eccPrimeSquared * pow(cos(phi1Rad), 2);
                            double R1 = a * (1.0 - eccSquared)
                                    / pow(1.0 - (eccSquared * pow(sin(phi1Rad), 2)), 1.5);
                            double D = x;
                            double LongOrigin = ((zone - 1) * 6) - 180 + 3;

                            lat = phi1Rad
                                    - ((N1 * tan(phi1Rad) / R1)
                                       * (D * D / 2.0
                                          - (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1
                                             - 9.0 * eccPrimeSquared)
                                                  * D * D * D * D / 24.0
                                          + (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1
                                             - 252.0 * eccPrimeSquared - 3.0 * C1 * C1)
                                                  * pow(D, 6) / 720.0));
                            lat = lat * 180.0 / M_PI;
                            lon = (D - (1.0 + 2.0 * T1 + C1) * pow(D, 3) / 6.0
                                   + (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1
                                      + 8.0 * eccPrimeSquared + 24.0 * T1 * T1)
                                           * pow(D, 5) / 120.0)
                                    / cos(phi1Rad);
                            lon = LongOrigin + (lon * 180.0 / M_PI);
                        };

                        processLogLines(
                                m_currentLogFile,
                                acId,
                                nullptr,
                                [&]([[maybe_unused]] const QString &timeStr,
                                    const QString &msgNameStr,
                                    const QString &dataPart,
                                    const QRegularExpression &spaceRe) {
                                    if (msgNameStr == targetMsg) {
                                        QStringList values;
                                        if (!dataPart.isEmpty()) {
                                            values = dataPart.split(spaceRe, Qt::SkipEmptyParts);
                                        }
                                        if (!isUtm
                                            && values.size() > std::max({latIdx, lonIdx, altIdx})) {
                                            double lat = values[latIdx].toDouble() * latScale;
                                            double lon = values[lonIdx].toDouble() * lonScale;
                                            double alt = altIdx != -1
                                                    ? values[altIdx].toDouble() * altScale
                                                    : 0.0;
                                            kmlCoords += QString::number(lon, 'f', 6)
                                                    + QStringLiteral(",")
                                                    + QString::number(lat, 'f', 6)
                                                    + QStringLiteral(",")
                                                    + QString::number(alt, 'f', 6)
                                                    + QStringLiteral(" ");
                                        } else if (isUtm
                                                   && values.size() > std::max({utmEastIdx,
                                                                                utmNorthIdx,
                                                                                utmZoneIdx,
                                                                                altIdx})) {
                                            double utmEast = values[utmEastIdx].toDouble() / 100.0;
                                            double utmNorth
                                                    = values[utmNorthIdx].toDouble() / 100.0;
                                            int utmZone = values[utmZoneIdx].toInt();
                                            double alt = altIdx != -1
                                                    ? values[altIdx].toDouble() * altScale
                                                    : 0.0;

                                            if (utmZone > 0 && alt > 0) {
                                                double lat;
                                                double lon;
                                                utm2deg(utmEast, utmNorth, utmZone, lat, lon);
                                                kmlCoords += QString::number(lon, 'f', 6)
                                                        + QStringLiteral(",")
                                                        + QString::number(lat, 'f', 6)
                                                        + QStringLiteral(",")
                                                        + QString::number(alt, 'f', 6)
                                                        + QStringLiteral(" ");
                                            }
                                        }
                                    }
                                });

                        QFile kmlFile(fileName);
                        if (kmlFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                            // QColor c = getNextSaturatedColor();
                            // QString kmlColor = QString("%1%2%3%4")
                            //     .arg(c.alpha(), 2, 16, QLatin1Char('0'))
                            //     .arg(c.blue(), 2, 16, QLatin1Char('0'))
                            //     .arg(c.green(), 2, 16, QLatin1Char('0'))
                            //     .arg(c.red(), 2, 16, QLatin1Char('0'));

                            QTextStream out(&kmlFile);
                            out << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
                            out << "  <Document>\n";
                            out << "    <name>" << logName << "_" << acId << "</name>\n";
                            out << "    <Placemark>\n";
                            out << "      <name>" << logName << "</name>\n";
                            out << "      <Style>\n";
                            out << "        <LineStyle>\n";
                            // out << "          <color>" << kmlColor << "</color>\n";//New option
                            // to you gusto
                            out << "          <color>ff0000ff</color>\n"; // Red color like in
                                                                          // original code
                            out << "          <width>2</width>\n";
                            out << "        </LineStyle>\n";
                            out << "      </Style>\n";
                            out << "      <LineString>\n";
                            out << "        <altitudeMode>absolute</altitudeMode>\n";
                            out << "        <coordinates>\n";
                            out << "          " << kmlCoords << "\n";
                            out << "        </coordinates>\n";
                            out << "      </LineString>\n";
                            out << "    </Placemark>\n";
                            out << "  </Document>\n";
                            out << "</kml>\n";
                        }
                    });

            QAction *exportCsvAction = acMenu->addAction(QStringLiteral("Export CSV"));
            connect(exportCsvAction,
                    &QAction::triggered,
                    this,
                    [this, acId, logName, dictFields, msgs]() {
                        QDialog dialog(this);
                        dialog.setWindowTitle(tr("Export CSV - %1").arg(acId));
                        dialog.resize(500, 600);
                        QVBoxLayout *layout = new QVBoxLayout(&dialog);

                        QTreeWidget *tree = new QTreeWidget(&dialog);
                        tree->setHeaderLabel(QStringLiteral("Messages and Fields"));
                        layout->addWidget(tree);

                        for (const QString &msgName : msgs) {
                            if (dictFields.contains(msgName)) {
                                QTreeWidgetItem *msgItem = new QTreeWidgetItem(tree);
                                msgItem->setText(0, msgName);
                                msgItem->setFlags(msgItem->flags() | Qt::ItemIsUserCheckable
                                                  | Qt::ItemIsAutoTristate);
                                msgItem->setCheckState(0, Qt::Unchecked);

                                const QStringList &fields = dictFields.value(msgName);
                                for (const QString &fieldName : fields) {
                                    QTreeWidgetItem *fieldItem = new QTreeWidgetItem(msgItem);
                                    fieldItem->setText(0, fieldName);
                                    fieldItem->setFlags(fieldItem->flags()
                                                        | Qt::ItemIsUserCheckable);
                                    fieldItem->setCheckState(0, Qt::Unchecked);
                                }
                            }
                        }

                        QDialogButtonBox *buttonBox = new QDialogButtonBox(
                                QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
                        buttonBox->button(QDialogButtonBox::Ok)->setText(QStringLiteral("Export"));

                        QHBoxLayout *delimLayout = new QHBoxLayout();
                        QLabel *delimLabel = new QLabel(QStringLiteral("Delimiter:"));
                        QComboBox *delimCombo = new QComboBox();
                        delimCombo->addItem(QStringLiteral("Comma (,)"), ",");
                        delimCombo->addItem(QStringLiteral("TAB"), "\t");
                        delimCombo->addItem(QStringLiteral("SPACE"), " ");
                        delimCombo->addItem(QStringLiteral("Semicolon (;)"), ";");
                        delimLayout->addWidget(delimLabel);
                        delimLayout->addWidget(delimCombo);
                        delimLayout->addStretch();
                        layout->addLayout(delimLayout);

                        layout->addWidget(buttonBox);
                        connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
                        connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

                        if (dialog.exec() == QDialog::Accepted) {
                            QString delimiter = delimCombo->currentData().toString();
                            QString docsLocation = QStandardPaths::writableLocation(
                                    QStandardPaths::DocumentsLocation);
                            if (docsLocation.isEmpty())
                                docsLocation = QFileInfo(m_currentLogFile).path();
                            QString defaultName
                                    = QDir(docsLocation)
                                              .filePath(logName + QStringLiteral("_") + acId
                                                        + QStringLiteral("_export.csv"));
                            QString outFileName;
                            {
                                StderrBlocker blocker;
                                outFileName = QFileDialog::getSaveFileName(
                                        this, tr("Save CSV"), defaultName, tr("CSV Files (*.csv)"));
                            }
                            if (outFileName.isEmpty())
                                return;

                            QMap<QString, QList<int>> selectedFields;
                            QMap<QString, QStringList> selectedFieldNames;

                            int totalCols = 0;
                            QStringList headerCols;
                            headerCols << QStringLiteral("Time");

                            for (int i = 0; i < tree->topLevelItemCount(); ++i) {
                                QTreeWidgetItem *msgItem = tree->topLevelItem(i);
                                QString msgName = msgItem->text(0);
                                for (int j = 0; j < msgItem->childCount(); ++j) {
                                    QTreeWidgetItem *fieldItem = msgItem->child(j);
                                    if (fieldItem->checkState(0) == Qt::Checked) {
                                        selectedFields[msgName].append(j);
                                        QString cName = msgName + QStringLiteral(".")
                                                + fieldItem->text(0);
                                        selectedFieldNames[msgName].append(cName);
                                        headerCols << cName;
                                        totalCols++;
                                    }
                                }
                            }
                            if (totalCols == 0) {
                                QMessageBox::information(this,
                                                         tr("Export CSV"),
                                                         tr("No fields selected for export."));
                                return;
                            }

                            QProgressDialog progress(tr("Exporting CSV..."),
                                                     tr("Cancel"),
                                                     0,
                                                     QFileInfo(m_currentLogFile).size(),
                                                     this);
                            progress.setWindowModality(Qt::WindowModal);
                            progress.setMinimumDuration(100);

                            doExportCsv(acId,
                                        m_currentLogFile,
                                        outFileName,
                                        selectedFields,
                                        selectedFieldNames,
                                        headerCols,
                                        delimiter,
                                        &progress);

                            QMessageBox::information(
                                    this, tr("Export CSV"), tr("CSV Export Complete!"));
                        }
                    });

            if (autoExportCsv) {
                QString docsLocation
                        = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
                if (docsLocation.isEmpty())
                    docsLocation = QFileInfo(m_currentLogFile).path();
                QString outFileName = QDir(docsLocation)
                                              .filePath(logName + QStringLiteral("_") + acId
                                                        + QStringLiteral("_export.csv"));
                QMap<QString, QList<int>> selectedFields;
                QMap<QString, QStringList> selectedFieldNames;
                int totalCols = 0;
                QStringList headerCols;
                headerCols << QStringLiteral("Time");

                for (const QString &msgName : std::as_const(msgs)) {
                    if (dictFields.contains(msgName)) {
                        const QStringList &fields = dictFields.value(msgName);
                        for (int j = 0; j < fields.size(); ++j) {
                            selectedFields[msgName].append(j);
                            QString cName = msgName + QStringLiteral(".") + fields[j];
                            selectedFieldNames[msgName].append(cName);
                            headerCols << cName;
                            totalCols++;
                        }
                    }
                }

                if (totalCols > 0) {
                    doExportCsv(acId,
                                m_currentLogFile,
                                outFileName,
                                selectedFields,
                                selectedFieldNames,
                                headerCols,
                                QStringLiteral(","));
                }
            }
        }

        while (QApplication::overrideCursor()) {
            QApplication::restoreOverrideCursor();
        }
    }
};

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
    // QCoreApplication::setOrganizationName("paparazzi"); // only for settings, not really relevant
    // here
    //  Follow XDG spec for desktop integration and use a fixed name to ensure the .desktop file is
    //  correctly associated with the app
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_logplotter"));
    QCoreApplication::setApplicationName(QStringLiteral("Paparazzi log plotter"));

    // Suppress Wayland text input garbage warnings, ybe Mutter dev get their act together one day
    // and fix this upstream, but until then, this is the cleanest solution to avoid spamming the
    // console with GTK criticals when opening native dialogs on Wayland.
    qputenv("QT_LOGGING_RULES", "qt.qpa.wayland.textinput=false");

    QApplication app(argc, argv);

    // app.setApplicationDisplayName(QStringLiteral("Log Plotter"));

    QString iconPath = QStringLiteral(":/penguin_icon_log.png");
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(QApplication::desktopFileName(),
                                   QStringLiteral("Paparazzi log plotter"),
                                   QStringLiteral("Log plotter for telemetry messages"),
                                   iconPath,
                                   QStringLiteral("paparazzi-logplotter"));

    // Apply the custom proxy style to the application.
    // We pass app.style() so it inherits all the default OS/Wayland drawing
    // behavior, simply layering our palette override on top.
    QApplication::setStyle(new EditorLighteningStyle(QApplication::style()));

    QApplication::setWindowIcon(icon);

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Paparazzi Log Plotter"));
    parser.addHelpOption();
    // Use manual version option to avoid "-v" conflict with verbose
    QCommandLineOption versionOption(QStringList() << QStringLiteral("version"),
                                     QStringLiteral("Displays version information."));
    parser.addOption(versionOption);

    QCommandLineOption exportCsvOption(
            QStringList() << QStringLiteral("export_csv"),
            QStringLiteral("Export in CSV in batch mode according to saved preferences."));
    parser.addOption(exportCsvOption);

    QCommandLineOption verboseOption(QStringList()
                                             << QStringLiteral("v") << QStringLiteral("verbose"),
                                     QStringLiteral("Verbose mode."));
    parser.addOption(verboseOption);

    parser.addPositionalArgument(QStringLiteral("logs"),
                                 QStringLiteral("Log files to open."),
                                 QStringLiteral("[log files...]"));

    // Parse the command line arguments
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

    if (parser.isSet(versionOption)) {
        parser.showVersion();
        return 0;
    }

    bool exportCsv = parser.isSet(exportCsvOption);
    bool verbose = parser.isSet(verboseOption);
    QStringList logFiles = parser.positionalArguments();

    if (verbose) {
        qDebug() << "Verbose mode enabled.";
        qDebug() << "Log files to process:" << logFiles;
    }

    if (logFiles.isEmpty()) {
        LogPlotterWindow *window = new LogPlotterWindow();
        window->setWindowIcon(icon);
        window->setAttribute(Qt::WA_DeleteOnClose);
        window->show();
    } else {
        for (const QString &argFile : std::as_const(logFiles)) {
            // Ignore arguments that sneak through Qt arg parsing like wayland parameters just in
            // case
            if (argFile == QLatin1String("--platform"))
                continue;

            LogPlotterWindow *window = new LogPlotterWindow();
            window->setWindowIcon(icon);
            window->setAttribute(Qt::WA_DeleteOnClose);
            window->loadLogFile(argFile, exportCsv);

            if (!exportCsv) {
                window->show();
            }
        }
    }

    if (exportCsv) {
        // Mock OCaml behavior where CSV export skips GUI loop execution
        QApplication::processEvents();
        return 0;
    }

    return QApplication::exec();
}

#include "logplotter.moc"
