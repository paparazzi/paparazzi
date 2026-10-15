/**
 * @file plotter_common.h
 * @brief Common plotting and rendering definitions for the Paparazzi Log Plotter.
 *
 * @details This file implements dynamic color generation algorithms to assure contrast
 * between numerous chart curves, along with an overlay-based active legend. By rendering
 * an overlay widget rather than relying on `QChart`'s native layout-bound legend,
 * this implementation saves critical screen real estate while bypassing Qt's
 * restrictive layout clipping behaviors in dense UI contexts.
 */
#pragma once
#include <algorithm>
#include <cmath>

#include <QChart>
#include <QColor>
#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineSeries>
#include <QObject>
#include <QSizePolicy>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

static inline int g_colorIndex = 0;

/**
 * @brief Generates a visually distinct color for a new curve using the Golden Angle.
 *
 * @details Employs the "Golden Angle" (137.508 degrees) method across the HSV color space.
 * This guarantees that successive colors are mathematically spread as far apart
 * as possible on the color wheel without hardcoding a finite palette loop.
 * This strategy naturally handles logs with an indeterminate, high number of variables.
 *
 * @return QColor A highly saturated, perceptually distinct color.
 */
// For real classic behaviour the first colors should be:
// "red"; "blue"; "green"; "orange"; "purple"; "magenta"
// "FF0000"; "0000FF"; "00FF00"; "FFA500"; "800080"; "FF00FF"
// Not implemented this full classic behaviour but if needed feel free
// to add the code to return these specific colors for the first 6 calls
// before switching to the golden angle method for 100% classic behavior.
static inline QColor getNextSaturatedColor()
{
    double h = std::fmod(g_colorIndex * 137.508, 360.0);
    g_colorIndex++;
    return QColor::fromHsvF(h / 360.0, 1.0, 1.0);
}

/**
 * @class ChartLegendManager
 * @brief Manages a semi-transparent floating legend over the QChartView.
 *
 * @details Bypasses `QChart::legend()`, which normally shrinks the chart geometry to fit
 * its data. This class injects an absolutely-positioned overlay widget inside the
 * parent view, polling the series tail-values locally so users can read live data
 * without hovering/tooltip delays.
 */
class ChartLegendManager : public QObject
{
public:
    /**
     * @brief Constructs the overlay legend infrastructure.
     * @param chart The target QChart containing the visual curves.
     * @param overlayParent The parent viewport (must be QChartView window) where the overlay
     * resides.
     */
    ChartLegendManager(QChart *chart, QWidget *overlayParent)
        : QObject(overlayParent)
        , m_chart(chart)
    {
        m_legendOverlay = new QWidget(overlayParent);
        m_legendOverlay->setObjectName("LegendOverlay");

        // Pass mouse events directly to the chart underneath so panning/zooming
        // works seamlessly even if the mouse initiates a drag over the legend.
        m_legendOverlay->setAttribute(Qt::WA_TransparentForMouseEvents);
        m_legendOverlay->setStyleSheet(
                QStringLiteral("#LegendOverlay { background-color: rgba(255, 255, 255, 220); "
                               "border: none; border-radius: 4px; } * { color: black; }"));

        m_legendLayout = new QVBoxLayout(m_legendOverlay);
        m_legendLayout->setContentsMargins(0, 0, 0, 0);
        m_legendLayout->setSpacing(0);
        m_legendOverlay->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        m_legendOverlay->hide();

        m_legendUpdateTimer = new QTimer(this);
        m_legendUpdateTimer->setInterval(200);
        connect(m_legendUpdateTimer,
                &QTimer::timeout,
                this,
                &ChartLegendManager::onLegendRefreshTimeout);
        m_legendUpdateTimer->start();

        overlayParent->installEventFilter(this);
    }

    /** @brief Returns pointer to the floating overlay widget natively. */
    QWidget *overlay() const { return m_legendOverlay; }

    /** @brief Marks the legend to be fully stripped and rebuilt sequentially. */
    void requestRefresh() { m_legendNeedsRefresh = true; }

    /**
     * @brief Entirely purges the local layout cache and redraws the UI from the chart series list.
     *
     * @details This is an expensive operation due to dynamic memory re-allocations of QLabels
     * and widgets. It should ONLY be invoked when a curve is strictly inserted or destroyed
     * (topology change), never during generic repaint or zoom cycles.
     */
    void updateLegendPosition()
    {
        if (!m_chart || !m_legendOverlay || !m_legendLayout)
            return;

        // Disable Qt's default internal geometry-eating legend
        m_chart->legend()->hide();

        m_legendOverlay->hide();

        // Unsafe to delete widgets immediately via pointer referencing.
        // We use takeAt to unlink layout ownership before calling deleteLater
        // to gracefully release Qt's asynchronous object tree.
        QLayoutItem *item;
        while ((item = m_legendLayout->takeAt(0)) != nullptr) {
            if (QWidget *widget = item->widget()) {
                widget->hide();
                widget->deleteLater();
            }
            delete item;
        }

        auto seriesList = m_chart->series();
        if (seriesList.isEmpty()) {
            return;
        }

        for (auto *s : std::as_const(seriesList)) {
            QLineSeries *ls = qobject_cast<QLineSeries *>(s);
            if (ls) {
                QWidget *rowWidget = new QWidget();
                rowWidget->setStyleSheet(QStringLiteral("background: transparent; border: none;"));
                QHBoxLayout *rowLayout = new QHBoxLayout(rowWidget);
                rowLayout->setContentsMargins(4, 2, 4, 2);
                rowLayout->setSpacing(5);

                QLabel *colorBox = new QLabel();
                QString colorMsg = ls->pen().color().name();
                colorBox->setStyleSheet(
                        QStringLiteral("background-color: %1; border: none;").arg(colorMsg));

                double latestVal = 0.0;
                if (ls->count() > 0) {
                    // Extract only the tail-end Y node as a visual data-snapshot.
                    latestVal = ls->at(ls->count() - 1).y();
                }
                QLabel *textLbl = new QLabel(
                        QStringLiteral("%1 : %2").arg(ls->name()).arg(latestVal, 0, 'f', 4));
                textLbl->setStyleSheet(
                        QStringLiteral("color: black; border: none; background: transparent;"));
                textLbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

                int textHeight = textLbl->fontMetrics().height();
                int boxSize = textHeight;
                colorBox->setFixedSize(boxSize, boxSize);

                rowLayout->addWidget(textLbl, 1);
                rowLayout->addWidget(colorBox, 0);

                m_legendLayout->addWidget(rowWidget);
                rowWidget->show();
            }
        }

        triggerRelayout();
    }

    /**
     * @brief Recomputes viewport metrics to map the absolute location of the floating legend.
     *
     * @details Relies on explicit math positioning mapped defensively to the `parentWidget`.
     * If handled as a standard layout addition, `QChartView` internally absorbs grid spaces
     * rather than floating dynamically as an overlay.
     */
    void triggerRelayout()
    {
        if (m_legendOverlay && m_legendOverlay->parentWidget()) {
            if (m_legendLayout->count() == 0) {
                m_legendOverlay->hide();
                return;
            }

            m_legendOverlay->show();

            m_legendLayout->invalidate();
            m_legendLayout->activate();
            m_legendOverlay->adjustSize();

            int pWidth = m_legendOverlay->parentWidget()->width();
            int wWidth = m_legendOverlay->width();

            // Lock position strictly to the Top-Right corner with a forced 15px padding margin.
            m_legendOverlay->move(std::max(0, pWidth - wWidth - 15), 15);

            m_legendOverlay->raise();
            m_legendOverlay->repaint();
        }
    }

protected:
    /**
     * @brief Intercepts generic events to handle automated snapping logic when the window is
     * resized.
     * @param obj The triggering Object
     * @param event The triggered Event payload
     * @return true if filtering intercepts the event natively.
     */
    bool eventFilter(QObject *obj, QEvent *event) override
    {
        // Only react to legitimate resizing limits so that the floating widget correctly snaps
        // layout coordinates
        if (event->type() == QEvent::Resize && m_legendOverlay
            && obj == m_legendOverlay->parentWidget()) {
            triggerRelayout();
        }
        return QObject::eventFilter(obj, event);
    }

private:
    /**
     * @brief A timer-based execution core to dynamically alter textual contents.
     *
     * @details Avoids catastrophic frame drops by strictly updating label texts
     * rapidly (at 200ms bounds) instead of allocating/tearing-down labels.
     * Since QLabels maintain stable rendering textures, replacing the `setText`
     * safely acts as an economical UI buffer.
     */
    void onLegendRefreshTimeout()
    {
        if (!m_legendOverlay || !m_legendLayout)
            return;
        auto seriesList = m_chart->series();

        // If series array size mismatches layout rows, a series was destroyed/added
        // outside standard handling pathways, so we demand a full architectural layout reset.
        if (m_legendLayout->count() != seriesList.size()) {
            updateLegendPosition();
            return;
        }
        for (int i = 0; i < seriesList.size(); ++i) {
            QLineSeries *ls = qobject_cast<QLineSeries *>(seriesList[i]);
            if (ls) {
                double latestVal = 0.0;
                if (ls->count() > 0) {
                    latestVal = ls->at(ls->count() - 1).y();
                }
                QLayoutItem *item = m_legendLayout->itemAt(i);
                if (item && item->widget()) {
                    QWidget *rowWidget = item->widget();
                    QHBoxLayout *l = qobject_cast<QHBoxLayout *>(rowWidget->layout());
                    // Update only if layout structure matches assertions
                    if (l && l->count() >= 2) {
                        QLabel *lbl = qobject_cast<QLabel *>(l->itemAt(0)->widget());
                        if (lbl) {
                            lbl->setText(QStringLiteral("%1 : %2")
                                                 .arg(ls->name())
                                                 .arg(latestVal, 0, 'f', 4));
                        }
                    }
                }
            }
        }
    }

    QChart *m_chart;
    QWidget *m_legendOverlay;
    QVBoxLayout *m_legendLayout;
    QTimer *m_legendUpdateTimer;
    bool m_legendNeedsRefresh = true;
};
