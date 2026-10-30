/**
 * @file os_desktop_utils.h
 * @brief Self-contained desktop-integration and UX helpers for Paparazzi Qt tools.
 *
 * @details This header-only component bundles three independent, dependency-free helpers that
 * every Paparazzi Qt application (GCS, messages, gaia, plotter, log plotter, replay, ...) can
 * reuse verbatim by simply including this header:
 *
 *  - installLinuxDesktopIntegration() auto-generates the XDG desktop rules (a `.desktop`
 *    launcher and a hicolor theme icon) directly from the running binary, at runtime, so the
 *    application integrates into GNOME / Wayland / Ubuntu Dock launchers without a system-wide
 *    install or an elevated (`sudo`) build step.
 *
 *  - EditorLighteningStyle is a QProxyStyle that lightens the otherwise unreadably dark
 *    background of text-input widgets under aggressive dark system themes, per-widget and
 *    without disturbing any other widget.
 *
 *  - StderrBlocker is an RAII guard that silences the harmless `Gdk-CRITICAL` assertion spam
 *    GTK prints to stderr when a Qt/Wayland application opens a native file dialog.
 *
 * Every member is inline / header-only, so this file may be included from multiple translation
 * units without violating the One Definition Rule.
 */
#ifndef OS_DESKTOP_UTILS_H
#define OS_DESKTOP_UTILS_H

// Core / I/O headers used by installLinuxDesktopIntegration().
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QIODevice>
#include <QProcess>
#include <QStandardPaths>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QtGlobal> // Q_OS_LINUX, Q_OS_UNIX, Q_UNUSED, qEnvironmentVariable

// Widget / styling headers used by EditorLighteningStyle (defined below).
#include <QAbstractSpinBox>
#include <QColor>
#include <QLineEdit>
#include <QPalette>
#include <QPlainTextEdit>
#include <QProxyStyle>
#include <QTextEdit>
#include <QWidget>

// POSIX headers used by StderrBlocker (file-descriptor redirection).
#ifdef Q_OS_UNIX
#include <cstdio> // std::fflush, stderr
#include <fcntl.h> // open, O_WRONLY, O_CLOEXEC
#include <unistd.h> // dup, dup2, close, STDERR_FILENO
#endif

/**
 * @brief Bootstraps a Linux `.desktop` file dynamically so the desktop manager recognizes the app.
 *
 * @param appDesktopFileName The base file name for the `.desktop` file (without extension). This
 *                           must match the value passed to QGuiApplication::setDesktopFileName()
 *                           so that the Wayland app_id / X11 WM_CLASS of the live window binds to
 *                           the generated launcher.
 * @param displayName        The human readable name displayed by the launcher.
 * @param appComment         A short tooltip explanation displayed by the desktop launcher.
 * @param iconResourcePath   Path to the icon to publish. A Qt resource path (e.g. ":/app.png")
 *                           is accepted as well as a regular filesystem path.
 * @param startupWMClass     The `StartupWMClass` value used to bind window groups to this entry.
 *
 * @details
 * The launcher and icon are written into the per-user XDG locations
 * (`QStandardPaths::ApplicationsLocation` and the hicolor icon theme), which avoids the
 * "Permission denied" issues of the global `/usr/share/applications` folder and keeps the helper
 * compatible with strict sandbox models (Flatpak / Snap / AppImage).
 *
 * @note If the icon resource is missing, the launcher falls back to the default generic system
 * icon.
 */
inline void installLinuxDesktopIntegration(const QString &appDesktopFileName,
                                           const QString &displayName,
                                           const QString &appComment,
                                           const QString &iconResourcePath,
                                           const QString &startupWMClass)
{
#ifdef Q_OS_LINUX
    // Resolve the absolute path of the running executable so the generated launcher
    // points back at this exact binary (works from a build tree, an install or an AppImage).
    QString exePath = QCoreApplication::arguments().at(0);
    if (!exePath.contains(QLatin1String("/"))) {
        exePath = QStandardPaths::findExecutable(exePath);
    } else {
        exePath = QDir::cleanPath(QDir().absoluteFilePath(exePath));
    }

    // Why wrap in bash and substitute the home directory?
    // Absolute host paths can confuse sandboxed environments that hardcode local mount points.
    // Replacing /home/<user>/ with ~/ and running through a bash exec lets the path be expanded
    // from the environment at launch time, keeping the launcher portable across machines.
    QString userName = qEnvironmentVariable("USER");
    if (!userName.isEmpty()
        && exePath.startsWith(QStringLiteral("/home/") + userName + QStringLiteral("/"))) {
        exePath.replace(0, (QStringLiteral("/home/") + userName).length(), QStringLiteral("~"));
        exePath = QStringLiteral("bash -c \"exec ") + exePath + QStringLiteral("\"");
    }

    // Follow the XDG base directory spec for file locations so we never need sudo and it keeps
    // working inside flatpak/snap/other containerized environments without special permissions.
    QString appsLocation = QStandardPaths::writableLocation(QStandardPaths::ApplicationsLocation);
    if (!appsLocation.isEmpty()) {
        QDir().mkpath(appsLocation);
        QString desktopFileName = appDesktopFileName + QStringLiteral(".desktop");
        QString desktopFilePath = appsLocation + QStringLiteral("/") + desktopFileName;
        QFile dfile(desktopFilePath);
        if (dfile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&dfile);
            out << "[Desktop Entry]\n"
                << "Version=1.0\n"
                << "Type=Application\n"
                << "Name=" << displayName << "\n"
                << "Comment=" << appComment << "\n"
                << "Exec=" << exePath << "\n"
                << "Icon=" << appDesktopFileName << "\n"
                << "Terminal=false\n"
                << "Categories=Education;Science;Robotics;\n"
                << "StartupNotify=true\n"
                << "StartupWMClass=" << startupWMClass << "\n";
            dfile.close();
        }

        // Publish the icon into the user hicolor theme under the launcher's name so that the
        // "Icon=<appDesktopFileName>" entry above resolves it.
        QString iconDir = QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation)
                + QStringLiteral("/icons/hicolor/128x128/apps");
        QDir().mkpath(iconDir);
        QString iconFilePath
                = iconDir + QStringLiteral("/") + appDesktopFileName + QStringLiteral(".png");
        if (QFile::exists(iconFilePath)) {
            QFile::remove(iconFilePath);
        }
        if (QFile::exists(iconResourcePath)) {
            QFile::copy(iconResourcePath, iconFilePath);
        }

        // Let the desktop databases catch up. These are best-effort: fired through Qt's native
        // cross-platform process API and silenced so a missing tool never disturbs the user.
        QProcess::startDetached(
                QStringLiteral("/bin/sh"),
                QStringList() << QStringLiteral("-c")
                              << QStringLiteral("update-desktop-database -q \"%1\" >/dev/null 2>&1")
                                         .arg(appsLocation));
        QString hicolorDir = QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation)
                + QStringLiteral("/icons/hicolor");
        QProcess::startDetached(
                QStringLiteral("/bin/sh"),
                QStringList() << QStringLiteral("-c")
                              << QStringLiteral(
                                         "gtk-update-icon-cache -q -t -f \"%1\" >/dev/null 2>&1")
                                         .arg(hicolorDir));
    }
#else
    Q_UNUSED(appDesktopFileName);
    Q_UNUSED(displayName);
    Q_UNUSED(appComment);
    Q_UNUSED(iconResourcePath);
    Q_UNUSED(startupWMClass);
#endif
}

/**
 * @class EditorLighteningStyle
 * @brief Lightens aggressively dark text-input widgets imposed by dark system themes.
 *
 * @details Many dark GTK/Qt palettes render @c QLineEdit / @c QTextEdit /
 * @c QPlainTextEdit / @c QAbstractSpinBox backgrounds so dark that their borders
 * and contents fail visibility checks. Rather than fighting Qt's global
 * stylesheet (which tends to strip native OS rendering), this proxy style hooks
 * the per-widget @c polish phase and raises only @c QPalette::Base to a readable
 * @c #3a3a3a on input widgets, leaving every other widget untouched. All members
 * are inline, so the class can be shared by including this header from multiple
 * translation units without violating the One Definition Rule.
 *
 * Usage: @code app.setStyle(new EditorLighteningStyle(app.style())); @endcode
 */
class EditorLighteningStyle : public QProxyStyle
{
public:
    // Inherit QProxyStyle's constructors (notably the one taking a base QStyle*).
    using QProxyStyle::QProxyStyle;

    /**
     * @brief Invoked automatically just before each widget is shown.
     * @param widget The widget being prepared for display.
     * @details Forces a lighter @c QPalette::Base on text-input widgets only.
     */
    void polish(QWidget *widget) override
    {
        // Always run the base class polish first.
        QProxyStyle::polish(widget);

        // Only adjust editable text/number input widgets.
        if (qobject_cast<QLineEdit *>(widget) || qobject_cast<QTextEdit *>(widget)
            || qobject_cast<QPlainTextEdit *>(widget) || qobject_cast<QAbstractSpinBox *>(widget)) {
            QPalette customPalette = widget->palette();
            customPalette.setColor(QPalette::Base, QColor(0x3a3a3a));
            // Apply only to this specific widget, not globally.
            widget->setPalette(customPalette);
        }
    }
};

/**
 * @class StderrBlocker
 * @brief RAII guard that temporarily redirects the process `stderr` to /dev/null for its scope.
 *
 * @details
 * WHY THIS EXISTS:
 * When a Qt application runs natively on Wayland and opens a *native* GTK file dialog (any of
 * the static QFileDialog helpers -- getOpenFileName(), getSaveFileName(), getExistingDirectory(),
 * getOpenFileNames() -- or a QFileDialog instance), the GTK file-chooser portal floods the
 * console with
 *     Gdk-CRITICAL **: gdk_wayland_window_set_dbus_properties_libgtk_only:
 *     assertion 'GDK_IS_WAYLAND_WINDOW (window)' failed
 * GTK is mishandling the foreign Wayland window handle that Qt passes it. The messages are
 * harmless but noisy, and they are written straight to the C `stderr` stream (through GLib's
 * g_log), so a Qt message handler (qInstallMessageHandler) cannot intercept them.
 *
 * WHY THIS APPROACH:
 * Rather than forcing `GDK_BACKEND=x11` for the whole process (which degrades Wayland
 * integration) or giving up the system-native file picker (QFileDialog::DontUseNativeDialog),
 * this guard redirects `stderr` to /dev/null for exactly the lifetime of the object -- i.e.
 * only while the modal dialog is on screen. Declare one on the stack right before opening the
 * dialog:
 *
 *     QString file;
 *     {
 *         StderrBlocker silence_gtk_wayland_spam;
 *         file = QFileDialog::getOpenFileName(this, ...);
 *     } // original stderr restored here
 *
 * The guard owns duplicated file descriptors, so it is deliberately non-copyable and
 * non-movable to guarantee each descriptor is released exactly once. On non-POSIX platforms it
 * compiles to a no-op. The redirection is process-wide for its (short) lifetime, so keep the
 * scope tight: any other stderr output produced while the dialog is open is suppressed too.
 */
class StderrBlocker
{
public:
    StderrBlocker()
    {
#ifdef Q_OS_UNIX
        std::fflush(stderr);
        // Keep a copy of the real stderr so it can be restored, and only redirect once BOTH the
        // saved copy and /dev/null were obtained -- otherwise stderr could never be put back.
        m_savedStderrFd = ::dup(STDERR_FILENO);
        m_devNullFd = ::open("/dev/null", O_WRONLY | O_CLOEXEC);
        if (m_savedStderrFd >= 0 && m_devNullFd >= 0) {
            ::dup2(m_devNullFd, STDERR_FILENO);
        }
#endif
    }

    ~StderrBlocker()
    {
#ifdef Q_OS_UNIX
        std::fflush(stderr);
        if (m_savedStderrFd >= 0) {
            ::dup2(m_savedStderrFd, STDERR_FILENO); // restore the original stderr
            ::close(m_savedStderrFd);
        }
        if (m_devNullFd >= 0) {
            ::close(m_devNullFd);
        }
#endif
    }

    StderrBlocker(const StderrBlocker &) = delete;
    StderrBlocker &operator=(const StderrBlocker &) = delete;
    StderrBlocker(StderrBlocker &&) = delete;
    StderrBlocker &operator=(StderrBlocker &&) = delete;

#ifdef Q_OS_UNIX
private:
    int m_savedStderrFd = -1;
    int m_devNullFd = -1;
#endif
};

#endif // OS_DESKTOP_UTILS_H
