/**
 * @file play.cpp
 * @brief Paparazzi Replay Engine (Native C++ Qt6 Implementation)
 *
 * @details
 * This application is a complete and robust architectural re-implementation of the legacy
 * OCaml-based "play", "play_core", and "play-nox" toolkits for the Paparazzi UAV framework.
 * It is engineered to load, parse, and broadcast Paparazzi .log telemetry datasets over an Ivy
 * software bus, simulating active aircraft streams natively to downstream Ground Control Station
 * entities (GCS) and visualization toolings.
 *
 * **Architectural Pillars:**
 * 1. **Zero-Allocation Parsing:** Relies on direct physical memory-mapped filesystem I/O
 * (`mmap()`). By abstaining from loading `QString` sequences onto the local heap, the application
 * routinely indexes gigabyte-scale datasets utilizing marginal active RAM (~20 bytes per telemetry
 * line lookup struct). Note: Fallback is provided if memory mapping fails.
 * 2. **Drift-Free Virtual Chronology:** Abandons standard single-shot OS sleep timeouts in favor of
 *    batched synchronous tracking (`QElapsedTimer`). The engine executes fixed 60Hz tick intervals
 * and bulk-transmits log sequences falling behind the active multiplied tracking clock, totally
 * eliminating long-term OS scheduler drift and UI stutter sequences even at heavy 100x+
 * multipliers.
 */

#include <QAbstractSlider>
#include <QAction>
#include <QApplication>
#include <QByteArray>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QDomDocument>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGuiApplication>
#include <QHash>
#include <QHBoxLayout>
#include <QIcon>
#include <QIODevice>
#include <QKeySequence>
#include <QLabel>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QObject>
#include <QProcess>
#include <QPushButton>
#include <QRegularExpression>
#include <QSet>
#include <QShowEvent>
#include <QSizePolicy>
#include <QSlider>
#include <QStandardPaths>
#include <QString>
#include <QStringList>
#include <QStringView>
#include <QStyle>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector>
#include <QWidget>

#include <algorithm>

#include <IvyQt/ivyqt.h>

// NOTE: this replay tool intentionally has NO dependency on the pprzlinkQt message library.
// Replaying only re-emits already-formatted log lines on the Ivy bus, and every .log embeds its
// own message dictionary (the <protocol> block, which storeMessages() writes back out). The only
// dictionary facts the engine needs -- which messages are telemetry vs ground, and which fields
// are integer-typed -- are read straight from messages.xml with QtXml (QDomDocument, already used
// here for the conf/protocol parsing). See initDictionary() / buildIntChecks().

#include "../include/os_desktop_utils.h"
#include "../include/pprz_version.h"

// ---------------------------------------------------------------------------
// Fast-forward / fast-backward seek step, in seconds. Since fixed at compile-time,
// change this one value to tune how far the  <<  /  >>  transport buttons jump.
// No runtime / dynamic setting involved, keeping the UI clean and focused on core playback
// controls.
// ---------------------------------------------------------------------------
#define PLAY_SEEK_STEP_SECONDS 10.0

/**
 * @brief Resolves the Paparazzi home directory
 *
 * @details Honors `$PAPARAZZI_HOME`; if unset, falls back to `$HOME/paparazzi`
 * exactly like the OCaml original (`Sys.getenv "HOME" // "paparazzi"`). As an
 * extra safety net (the OCaml code would raise an uncaught exception if `$HOME`
 * were also missing) we finally fall back to the current working directory so
 * headless/CI invocations never crash.
 */
static QString paparazziHome()
{
    QString home = qEnvironmentVariable("PAPARAZZI_HOME");
    if (!home.isEmpty())
        return home;
    const QString userHome = qEnvironmentVariable("HOME");
    if (!userHome.isEmpty())
        return userHome + QStringLiteral("/paparazzi");
    return QDir::currentPath();
}

/**
 * @brief Resolves the Paparazzi source tree, mirroring OCaml `Env.paparazzi_src`.
 *
 * @details Honors `$PAPARAZZI_SRC`; if unset, falls back to the system install
 * prefix `/usr/share/paparazzi` exactly like the OCaml original. This guarantees
 * `dump_flight_plan.out` resolves to a valid absolute path even when only
 * `$PAPARAZZI_HOME` is exported (previously an empty `$PAPARAZZI_SRC` produced the
 * broken root path `/sw/tools/generators/dump_flight_plan.out`).
 */
static QString paparazziSrc()
{
    QString src = qEnvironmentVariable("PAPARAZZI_SRC");
    if (!src.isEmpty())
        return src;
    return QStringLiteral("/usr/share/paparazzi");
}

/**
 * @struct LogIndex
 * @brief An ultra-lightweight structural pointer indexing telemetry frames sequentially.
 *
 * @details
 * Rather than instantiating heavily bloated object strings (`QString` naturally bounds internal
 * arrays, encoders, and meta layouts scaling around ~24+ bytes linearly per sequence), we track
 * only physical properties. If a e.g. 2GB raw file harbors ~20 million rows of data bounds, storing
 * an array of constructed strings would dynamically pull upwards of 12GB of operational virtual
 * memory space triggering Garbage Collection lockups. Using this struct, a dense 20-million
 * sequence requires approximately `sizeof(LogIndex) * 20_000_000` (approx. 400 MB) of indexing
 * capability linearly!
 */
struct LogIndex
{
    double time; ///< Primary chronological metric interpreted parsed during initialization mapping
                 ///< operations natively.
    qint64 offset; ///< Hard physical byte integer location matching strictly the start character of
                   ///< the parsed frame.
    int length; ///< Captured distance trailing the active newline carriage return dictating the
                ///< read scope limits purely.
};

/**
 * @class PlayCore
 * @brief The Central Logic framework handling File Tracking, Timers, XML Generation and Ivy Network
 * Broadcasts.
 *
 * @details
 * Implements a strict architectural partition. The core does not manage GUI states; it exclusively
 * manages operational mathematics, Ivy bus instantiations, memory pointers, and tracking logic,
 * enabling `play-nox` headless command-line deployments seamlessly tracking similar mechanisms to
 * active desktop Window setups natively.
 */
class PlayCore : public QObject
{
    Q_OBJECT
public:
    /**
     * @brief Initiates standard operational engine variables establishing high-precision ticking
     * states.
     */
    explicit PlayCore(QObject *parent = nullptr)
        : QObject(parent)

    {
        // Enforce the use of High-Resolution Precision timers requesting explicit priority against
        // backend OS schedulers This ensures interval drifting remains heavily controlled resolving
        // at ~60 Hz cycles mimicking a smooth operational framerate.
        m_tickTimer = new QTimer(this);
        m_tickTimer->setTimerType(Qt::PreciseTimer);
        m_tickTimer->setInterval(16);
        connect(m_tickTimer, &QTimer::timeout, this, &PlayCore::onTimeout);
    }

    ~PlayCore() override
    {
        if (m_mappedData) {
            m_dataFile.unmap(m_mappedData);
        }
        if (m_dataFile.isOpen()) {
            m_dataFile.close();
        }
    }

    void setIvyBus(const QString &busArg) { m_ivyBusArg = busArg; }
    void setNoGui(bool noGui) { m_noGui = noGui; }

    /**
     * @brief Overrides the consumer message dictionary used to validate outgoing payloads.
     * @param path An explicit messages.xml; empty keeps the default
     * ($PAPARAZZI_HOME/var/messages.xml).
     * @details Lets a user point payload int-validation at the exact dictionary their GCS/server
     * runs, e.g. when replaying into a non-default or remote configuration. Must be called before
     * loadLog().
     */
    void setConsumerMessages(const QString &path) { m_consumerMessagesPath = path; }

    /**
     * @brief Interprets initial log payloads targeting configurations and indexing raw dataset
     * binaries.
     * @param xmlFile Target system path identifying the `.log` root tracking XML schema structure
     * natively.
     * @return True if parsing, environment linking, and dictionary processing completed without
     * fail.
     *
     * @details Discovers embedded `.data` files, validates automated decompression tools if data is
     * tightly bound, attempts High-Speed Memory Mapping (falling back robustly on chunk-reading
     * constraints), bounds dictionaries, and triggers configuration block extractions mimicking
     * baseline Paparazzi environments.
     */
    bool loadLog(const QString &xmlFile)
    {
        if (m_isPlaying)
            stop();

        // Unmap memory bounds ensuring overlapping Hot-Reload capabilities do not leak massive
        // active dataset bounds organically.
        if (m_mappedData) {
            m_dataFile.unmap(m_mappedData);
            m_mappedData = nullptr;
        }
        if (m_dataFile.isOpen()) {
            m_dataFile.close();
        }

        m_xmlFile = xmlFile;
        QFile file(xmlFile);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qWarning() << "Cannot open log file:" << xmlFile;
            return false;
        }

        QString content = file.readAll();
        file.close();

        // GUARANTEE: Legacy paparazzi logs could contain heavily misformatted XML with un-escaped
        // entities natively encoded into attributes. Used library strictly rejects these violating
        // standard definitions. Normalize explicitly! We only target strictly cased variants of
        // explicit layout keys and never `CaseInsensitive` (<control> vs <Control>)
        content.replace(QRegularExpression(QStringLiteral("<(Control|Shift|Alt)>")),
                        QStringLiteral("&lt;\\1&gt;"));
        // Only safely replace unescaped ampersands to avoid destroying valid tags (like <control>
        // blocks in flight plans)
        content.replace(
                QRegularExpression(QStringLiteral("&(?!(amp|lt|gt|quot|apos|#)[a-zA-Z0-9]*;)")),
                QStringLiteral("&amp;"));

        QString dataFileName = xmlFile;

        // Using robust RegEx same as from logplotter to find the data_file securely even if XML is
        // poorly formed root-wise
        QRegularExpression reDataFile(QStringLiteral("data_file=\"([^\"]+)\""));
        QRegularExpressionMatch matchDataFile = reDataFile.match(content);
        if (matchDataFile.hasMatch()) {
            dataFileName = matchDataFile.captured(1);
        } else if (!xmlFile.endsWith(QLatin1String(".data"), Qt::CaseInsensitive)) {
            qWarning() << "data_file property not found in log.";
            return false;
        }

        QFileInfo fi(xmlFile);
        QString dataFilePath = fi.absoluteDir().filePath(dataFileName);

        // Resolve a compressed data file the same way OCaml `Ocaml_tools.find_file` +
        // `open_compress` do: search the log's own directory for a known archive
        // extension and decompress it. Unlike the previous in-place `gunzip <file>`
        // (which DELETED the user's original archive), we stream via `-c` to a plain
        // sidecar file, leaving the original intact -- matching OCaml's `gunzip -c`.
        if (!QFile::exists(dataFilePath)) {
            struct Variant
            {
                const char *ext;
                const char *tool;
            };
            static const Variant variants[]
                    = {{".gz", "gunzip"}, {".Z", "gunzip"}, {".bz2", "bunzip2"}};
            for (const auto &v : variants) {
                const QString compressed = dataFilePath + v.ext;
                if (QFile::exists(compressed)) {
                    QProcess proc;
                    proc.setStandardOutputFile(dataFilePath, QIODevice::Truncate);
                    proc.start(v.tool, {"-c", compressed});
                    proc.waitForFinished(-1);
                    if (proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0) {
                        QFile::remove(dataFilePath); // discard any partial/garbage output
                        qWarning() << "Failed to decompress data file:" << compressed;
                    }
                    break;
                }
            }
        }

        m_dataFile.setFileName(dataFilePath);
        if (!m_dataFile.open(QIODevice::ReadOnly)) {
            qWarning() << "Cannot open data file:" << dataFilePath;
            return false;
        }

        m_log.clear();
        m_streamFrames.clear(); // invalidate any prior scrub index before re-indexing
        QSet<QString> acs;

        // Core Performance Optimization 1: System Memory Boundary Mappings mapping native file
        // tracks into explicit pointer references zeroing block cache arrays naturally.
        m_mappedData = m_dataFile.map(0, m_dataFile.size());

        if (m_mappedData) {
            const char *ptr = reinterpret_cast<const char *>(m_mappedData);
            qint64 totalSize = m_dataFile.size();
            qint64 currentPos = 0;

            while (currentPos < totalSize) {
                const char *lineStartPtr = ptr + currentPos;
                const char *nlPtr = static_cast<const char *>(
                        memchr(lineStartPtr, '\n', totalSize - currentPos));
                int lineLen = nlPtr ? (nlPtr - lineStartPtr) : (totalSize - currentPos);

                int effectiveLineLen = lineLen;
                if (effectiveLineLen > 0 && lineStartPtr[effectiveLineLen - 1] == '\r') {
                    effectiveLineLen--; // Windows/CRLF stripping protecting parser validity
                                        // strictly
                }

                if (effectiveLineLen > 0) {
                    processLineForIndex(lineStartPtr, effectiveLineLen, currentPos, acs);
                }

                currentPos += lineLen + (nlPtr ? 1 : 0);
            }
        } else {
            // Core Performance Fallback: Chunked boundaries protecting constrained generic
            // environments failing OS large-file Mmap bindings dynamically.
            const int CHUNK_SIZE = 1048576; // 1MB chunks
            QByteArray buffer;
            qint64 fileOffset = 0;

            while (!m_dataFile.atEnd()) {
                QByteArray chunk = m_dataFile.read(CHUNK_SIZE);
                if (chunk.isEmpty())
                    break;

                buffer.append(chunk);
                // GUARANTEE: Prevent infinitely expanding buffers resulting in OOM on malformed
                // binaries
                if (buffer.size() > 50 * 1024 * 1024) {
                    qWarning() << "Malformed log file: excessively long line strings detected. "
                                  "Halting stream parser.";
                    break;
                }
                int lineStart = 0;
                while (true) {
                    int nlIdx = buffer.indexOf('\n', lineStart);
                    if (nlIdx == -1)
                        break;

                    int lineLen = nlIdx - lineStart;
                    int effectiveLineLen = lineLen;
                    if (effectiveLineLen > 0
                        && buffer.at(lineStart + effectiveLineLen - 1) == '\r') {
                        effectiveLineLen--;
                    }

                    if (effectiveLineLen > 0) {
                        const char *lineData = buffer.constData() + lineStart;
                        processLineForIndex(
                                lineData, effectiveLineLen, fileOffset + lineStart, acs);
                    }
                    lineStart = nlIdx + 1;
                }
                buffer.remove(0, lineStart);
                fileOffset += lineStart;
            }
            if (buffer.length() > 0) {
                int effectiveLineLen = buffer.length();
                if (buffer.at(effectiveLineLen - 1) == '\r')
                    effectiveLineLen--;
                if (effectiveLineLen > 0) {
                    processLineForIndex(buffer.constData(), effectiveLineLen, fileOffset, acs);
                }
            }
        }

        if (m_log.isEmpty()) {
            qWarning() << "No telemetry lines found in data file.";
            return false;
        }

        // GUARANTEE: Ensure absolute chronological integrity. Some logs reset or jump timelines.
        // std::lower_bound exhibits undefined behavior (crashing) if the timeline is un-ordered.
        std::sort(m_log.begin(), m_log.end(), [](const LogIndex &a, const LogIndex &b) {
            return a.time < b.time;
        });

        // Build the per-stream frame index now that m_log is time-sorted; this powers
        // O(log n) snapshot lookups for responsive live scrubbing (see broadcastSnapshot).
        buildStreamIndex();

        // Now parse the standard XML properties (Conf and Protocols). Many Paparazzi .log files
        // miss a single root element wrapper.
        QDomDocument doc;
        QString errorMsg;
        int errorLine = 0;
        int errorColumn = 0;
        bool ok = doc.setContent(content, &errorMsg, &errorLine, &errorColumn);
        if (!ok) {
            // Attempt to wrap it
            QString wrapped = QStringLiteral("<root>") + content + QStringLiteral("</root>");
            ok = doc.setContent(wrapped, &errorMsg, &errorLine, &errorColumn);
            if (!ok) {
                qWarning() << "XML Parse Error:" << errorMsg
                           << "at line:" << errorLine;
            }
        }

        QDomElement root = doc.documentElement();

        // Extract native configs and load internal routing dictionaries
        storeConf(root, acs);
        storeMessages(root);
        initDictionary();

        m_currentIndex = 0;
        m_virtualTime = m_log.first().time;
        emit logLoaded(m_log.first().time, m_log.last().time);

        qInfo() << QStringLiteral("Successfully indexed %1 telemetry frames.").arg(m_log.size());
        return true;
    }

    /**
     * @brief Binds explicit external Ivy bus boundaries matching defined local scope limitations
     * identically pushing system tracks cleanly.
     */
    void startIvy()
    {
        QString pprzName = QStringLiteral("Paparazzi replay");
        m_bus = new IvyQt(pprzName, QStringLiteral("READY"), this);
        QString domain = m_ivyBusArg;
        int port = 2010;
        if (domain.contains(':')) {
            QStringList parts = domain.split(':');
            domain = parts[0];
            port = parts[1].toInt();
        }
        m_bus->start(domain, port);

        // Establish native bindings pushing logical configurations mimicking active Paparazzi sim
        // limits modifying playback naturally tracking UI constraints
        m_bus->bindMessage(QStringLiteral("WORLD_ENV (.*)"), [this](Peer *, QStringList params) {
            if (m_timeScaleIdx >= 0 && !params.isEmpty()) {
                const QString &data = params[0];
                QStringList fields = data.split(' ', Qt::SkipEmptyParts);
                if (m_timeScaleIdx < fields.size()) {
                    bool ok;
                    double scale = fields[m_timeScaleIdx].toDouble(&ok);
                    if (ok && scale > 0.0) {
                        emit speedChangedByNetwork(scale);
                    }
                }
            }
        });
    }

    void setSpeed(double speed)
    {
        m_speed = std::max(0.001,
                           speed); // Protective absolute capping rejecting logic gaps strictly
    }

    /**
     * @brief Live-scrubbing seek: repositions the playback head to time `t` AND immediately
     * broadcasts a coherent state snapshot so any connected GCS reflects the exact aircraft
     * state at the scrubbed position -- the heart of responsive back-and-forward scrubbing.
     *
     * @details Three concerns are handled together:
     *  1. **Seek** -- an O(log n) binary search (`indexForTime`) positions the head; combined
     *     with the memory-mapped data this makes the jump itself effectively instantaneous on
     *     any log size.
     *  2. **Snapshot** -- `broadcastSnapshot` resends the latest frame of every distinct message
     *     stream at or before `t`. Sending only the single frame under the cursor would leave
     *     every other signal stale, so a per-stream snapshot is what makes the displayed state
     *     correct whether the user jumps forward or backward.
     *  3. **Clock safety** -- if a scrub happens mid-playback the elapsed accumulator is
     *     restarted so the next tick does not dump a huge delta and fast-forward.
     */
    void scrubTo(double t)
    {
        if (m_log.isEmpty())
            return; // Absolute protection against empty references
        m_currentIndex = indexForTime(t);
        m_virtualTime = m_log[m_currentIndex].time;
        if (m_isPlaying)
            m_elapsed.restart();
        broadcastSnapshot(m_virtualTime);
        emit timeUpdated(m_virtualTime);
    }

    void getBounds(double &start, double &end)
    {
        if (!m_log.isEmpty()) {
            start = m_log.first().time;
            end = m_log.last().time;
        } else {
            start = 0;
            end = 0;
        }
    }

    void play()
    {
        if (m_log.isEmpty())
            return;
        // If the head sits at (or past) the end of the log -- e.g. a previous run played
        // out completely -- rewind to the start so "Play" is never a silent no-op.
        if (m_currentIndex >= m_log.size()) {
            m_currentIndex = 0;
            emit timeUpdated(m_log.first().time);
        }
        m_isPlaying = true;
        m_virtualTime = m_log[m_currentIndex].time;
        m_elapsed.start();
        m_tickTimer->start();
        emit stateChanged(true);
    }

    void stop()
    {
        m_isPlaying = false;
        m_tickTimer->stop();
        emit stateChanged(false);
    }

    /** @brief Whether playback is currently active (used to auto-resume after a scrub). */
    bool isPlaying() const { return m_isPlaying; }

signals:
    void logLoaded(double, double);
    void timeUpdated(double);
    void speedChangedByNetwork(double);
    void finished();
    void stateChanged(bool);

private:
    /**
     * @brief Operational driver dictating native loop mechanisms bypassing OS drift issues
     * implicitly.
     *
     * @details
     * To accommodate robust simulation loops tracking 100x+ overrides locally driving Gigabyte
     * datasets gracefully:
     * 1. Check physical `QElapsedTimer` ms bounds organically passed generating precise
     * chronological deltas structurally tracking active hardware.
     * 2. Append values onto internal `virtualTime` constraints overriding software array
     * limitations reliably mapping true-to-life offsets purely.
     * 3. Send all consecutive payload lines strictly enclosed up to current tracked boundary
     * ensuring zero-event stalls organically!
     */
    void onTimeout()
    {
        if (!m_isPlaying || m_currentIndex >= m_log.size())
            return;

        qint64 ms = m_elapsed.restart();
        m_virtualTime += (ms / 1000.0) * m_speed;

        int messagesSent = 0;

        // Loop resolves strictly passing messages evaluating valid mapped structures locally
        // bounding transmission natively identically bypassing stall behaviors organically
        while (m_currentIndex < m_log.size() && m_log[m_currentIndex].time <= m_virtualTime) {
            // Broadcasting is delegated to sendFrame() so the identical wire format can be
            // reused by a future live-scrubbing seek path without duplicating any logic.
            sendFrame(m_log[m_currentIndex]);

            m_currentIndex++;
            messagesSent++;

            // Safety cap: Prevents permanent locking of the Main GUI render thread if multiplier
            // is intensely high on ultra dense datalogs.
            if (messagesSent >= 5000) {
                m_virtualTime = m_log[std::max(0, m_currentIndex - 1)].time;
                break;
            }
        }

        // Synchronize visual displays exclusively executing single loop-bound variables tracking
        // clean GUI events flawlessly minimizing CPU overhead naturally explicitly
        emit timeUpdated(m_virtualTime);

        if (m_currentIndex >= m_log.size()) {
            stop();
            if (m_noGui) {
                emit finished();
                QCoreApplication::quit();
            }
        }
    }

    /**
     * @brief Manual raw char pointer string sequence parser dictating array definitions without
     * issue.
     * @details Extracts values directly indexing chronological elements without executing complex
     * heap conversions enabling ultra low-latency evaluation variables natively.
     */
    void
    processLineForIndex(const char *lineData, int lineLen, qint64 fileOffset, QSet<QString> &acs)
    {
        int s1 = -1;
        int len1 = 0;
        int s2 = -1;
        int len2 = 0;

        for (int i = 0; i < lineLen; ++i) {
            if (lineData[i] != ' ' && lineData[i] != '\t' && lineData[i] != '\r') {
                if (s1 == -1) {
                    s1 = i;
                } else if (len1 > 0 && s2 == -1) {
                    s2 = i;
                }
            } else {
                if (s1 != -1 && s2 == -1) {
                    len1 = i - s1;
                } else if (s2 != -1 && len2 == 0) {
                    len2 = i - s2;
                    break;
                }
            }
        }
        if (s2 != -1 && len2 == 0)
            len2 = lineLen - s2;

        if (s1 != -1 && len1 > 0 && s2 != -1 && len2 > 0) {
            bool ok = false;
            double t = QByteArray::fromRawData(lineData + s1, len1).toDouble(&ok);
            // GUARANTEE: Filter out NaN/Infinity artifacts which fatally corrupt binary searches
            if (ok && std::isfinite(t)) {
                m_log.push_back({t, fileOffset, lineLen});
                QString ac = QString::fromUtf8(lineData + s2, len2);
                acs.insert(ac);
            }
        }
    }

    /**
     * @brief Isolates explicit payload arguments via direct String instantiations rapidly across
     * memory blocks.
     */
    static void
    extractMsgData(const char *data, int lineLen, QString &ac, QString &msgName, QString &msg)
    {
        int s1 = -1;
        int len1 = 0;
        int s2 = -1;
        int len2 = 0;
        int s3 = -1;
        int len3 = 0;
        for (int i = 0; i < lineLen; ++i) {
            if (data[i] != ' ' && data[i] != '\t' && data[i] != '\r') {
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
        if (s3 != -1 && len3 == 0)
            len3 = lineLen - s3;

        if (s2 != -1 && len2 > 0 && s3 != -1 && len3 > 0) {
            ac = QString::fromUtf8(data + s2, len2);
            msgName = QString::fromUtf8(data + s3, len3);
            msg = QString::fromUtf8(data + s3, lineLen - s3).trimmed();
        }
    }

    /**
     * @struct IntFieldCheck
     * @brief One integer-typed field that must be validated before a frame is broadcast.
     * @details `index` is the field's position in the message definition; `isArray` marks a
     * comma-separated array field whose every element must be a valid integer.
     */
    struct IntFieldCheck
    {
        int index;
        bool isArray;
    };

    /**
     * @brief True iff `s` is a plain decimal integer (optional sign + digits) -- exactly what the
     * (OCaml or other)/PprzLink consumers accept via int_of_string for integer-typed fields.
     */
    static bool isDecimalInteger(QStringView s)
    {
        if (s.isEmpty())
            return false;
        const int n = static_cast<int>(s.size());
        int i = 0;
        if (s[0] == QLatin1Char('+') || s[0] == QLatin1Char('-')) {
            if (n == 1)
                return false; // a lone sign is not an integer
            i = 1;
        }
        for (; i < n; ++i) {
            const QChar c = s[i];
            if (c < QLatin1Char('0') || c > QLatin1Char('9'))
                return false;
        }
        return true;
    }

    /**
     * @brief Validates a single integer field token; for array fields every comma-separated
     * element must itself be a valid decimal integer.
     */
    static bool validIntField(QStringView field, bool isArray)
    {
        if (!isArray)
            return isDecimalInteger(field);
        if (field.isEmpty())
            return false;
        const int m = static_cast<int>(field.size());
        int start = 0;
        for (int i = 0; i <= m; ++i) {
            if (i == m || field[i] == QLatin1Char(',')) {
                if (!isDecimalInteger(field.mid(start, i - start)))
                    return false;
                start = i + 1;
            }
        }
        return true;
    }

    /**
     * @brief Single-pass, allocation-free check that every integer-typed field of a payload holds
     * a valid integer.
     *
     * @details `msg` is "<MSGNAME> <f0> <f1> ..."; declared field i is token (i+1). `checks` is in
     * ascending field order. This mirrors how a consumer tokenizes the payload and applies
     * int_of_string, so a frame that would make a consumer throw Failure("int_of_string") is
     * detected here. Returns false if a required integer field is absent (the line is under-filled
     * versus the current definition -- which the consumer cannot parse either).
     */
    static bool payloadIntFieldsValid(const QVector<IntFieldCheck> &checks, const QString &msg)
    {
        if (checks.isEmpty())
            return true;
        const QStringView v(msg);
        const int n = static_cast<int>(v.size());
        int ci = 0;
        int targetTok = checks[0].index + 1;
        int tok = -1;
        int i = 0;
        while (i < n && ci < checks.size()) {
            while (i < n && (v[i] == QLatin1Char(' ') || v[i] == QLatin1Char('\t')))
                ++i;
            if (i >= n)
                break;
            const int start = i;
            while (i < n && v[i] != QLatin1Char(' ') && v[i] != QLatin1Char('\t'))
                ++i;
            ++tok;
            if (tok == targetTok) {
                if (!validIntField(v.mid(start, i - start), checks[ci].isArray))
                    return false;
                if (++ci < checks.size())
                    targetTok = checks[ci].index + 1;
            }
        }
        return ci >= checks.size(); // every integer field was present AND valid
    }

    /**
     * @brief Opens a Paparazzi messages.xml and returns its <protocol> root element, or a null
     * element if the file is missing or is not a protocol document. The QDomDocument that owns the
     * node tree is returned through @p docOut and must outlive any use of the returned element.
     *
     * @details A replay tool needs nothing more than plain XML here: the .log already embeds this
     * very dictionary, so QtXml (already a dependency) fully replaces the pprzlink message library.
     * setContent() ignores the SYSTEM DOCTYPE, so a `<!DOCTYPE protocol SYSTEM "messages.dtd">`
     * header never triggers an external-DTD fetch. This is like the past OCaml code's
     * `Xmlm.make_input` + `Xmlm.input` sequence, but with the robustness of Qt's forgiving parser
     * and the convenience of a live DOM tree to query.
     */
    static QDomElement openProtocol(const QString &path, QDomDocument &docOut)
    {
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly))
            return {};
        const QByteArray data = f.readAll();
        f.close();
        if (!docOut.setContent(data))
            return {};
        const QDomElement root = docOut.documentElement();
        if (root.tagName() != QLatin1String("protocol"))
            return {};
        return root;
    }

    /**
     * @brief Inserts the name of every <message> in msg_class @p className into @p out.
     * @details Mirrors pprzlink getMsgsForClass(name) -> message names, but tolerates a missing
     * class/message @c id (routing only needs the class<->name mapping, never the numeric ids).
     */
    static void collectClassMessageNames(const QDomElement &protocol,
                                         const QString &className,
                                         QSet<QString> &out)
    {
        for (QDomElement cls = protocol.firstChildElement(QStringLiteral("msg_class"));
             !cls.isNull();
             cls = cls.nextSiblingElement(QStringLiteral("msg_class"))) {
            if (cls.attribute(QStringLiteral("name")) != className)
                continue;
            for (QDomElement m = cls.firstChildElement(QStringLiteral("message")); !m.isNull();
                 m = m.nextSiblingElement(QStringLiteral("message"))) {
                const QString n = m.attribute(QStringLiteral("name"));
                if (!n.isEmpty())
                    out.insert(n);
            }
        }
    }

    /**
     * @brief Zero-based index of field @p fieldName within message @p msgName, or -1 if absent.
     * @details Document order is the field order, exactly how pprzlink numbered fields.
     */
    static int
    fieldIndex(const QDomElement &protocol, const QString &msgName, const QString &fieldName)
    {
        for (QDomElement cls = protocol.firstChildElement(QStringLiteral("msg_class"));
             !cls.isNull();
             cls = cls.nextSiblingElement(QStringLiteral("msg_class"))) {
            for (QDomElement m = cls.firstChildElement(QStringLiteral("message")); !m.isNull();
                 m = m.nextSiblingElement(QStringLiteral("message"))) {
                if (m.attribute(QStringLiteral("name")) != msgName)
                    continue;
                int idx = 0;
                for (QDomElement f = m.firstChildElement(QStringLiteral("field")); !f.isNull();
                     f = f.nextSiblingElement(QStringLiteral("field")), ++idx) {
                    if (f.attribute(QStringLiteral("name")) == fieldName)
                        return idx;
                }
                return -1; // message found, field not present
            }
        }
        return -1;
    }

    /**
     * @brief Precomputes, for one <message> element, the integer fields that must be validated
     * before broadcast. Stops at the first string/char[] field, because free text can contain
     * spaces and would misalign whitespace tokenization of everything after it.
     *
     * @details Reads each field's @c type attribute directly from messages.xml. A trailing
     * `[]`/`[N]` marks an array (every element must parse as an integer); the base name before it
     * selects the type. This reproduces pprzlink's FieldType classification without the library,
     * since the .log carries the very dictionary the player already parses.
     */
    static void buildIntChecks(const QDomElement &message,
                               QHash<QString, QVector<IntFieldCheck>> &out)
    {
        const QString msgName = message.attribute(QStringLiteral("name"));
        if (msgName.isEmpty())
            return;
        QVector<IntFieldCheck> checks;
        int i = 0;
        for (QDomElement f = message.firstChildElement(QStringLiteral("field")); !f.isNull();
             f = f.nextSiblingElement(QStringLiteral("field")), ++i) {
            const QString typeStr = f.attribute(QStringLiteral("type"));
            const int br = typeStr.indexOf(QLatin1Char('['));
            const bool isArray = (br != -1);
            const QString base = (br == -1) ? typeStr : typeStr.left(br);
            // A free-text field ends the safely-tokenisable prefix (pprzlink: STRING, or CHAR[]).
            if (base == QLatin1String("string") || (base == QLatin1String("char") && isArray)) {
                break;
            }
            if (base == QLatin1String("int8") || base == QLatin1String("int16")
                || base == QLatin1String("int32") || base == QLatin1String("uint8")
                || base == QLatin1String("uint16") || base == QLatin1String("uint32")) {
                checks.push_back(IntFieldCheck {i, isArray});
            }
        }
        if (!checks.isEmpty())
            out.insert(msgName, checks);
    }

    /**
     * @brief Class-aware guard: true if `msgName`'s payload is safe to broadcast. Logs a single
     * notice per offending message so a version-mismatched log is easy to diagnose without spamming
     * the log
     */
    bool frameIsBroadcastSafe(const QHash<QString, QVector<IntFieldCheck>> &checks,
                              const QString &msgName,
                              const QString &msg)
    {
        const auto it = checks.constFind(msgName);
        if (it == checks.constEnd())
            return true; // no integer fields to validate
        if (payloadIntFieldsValid(it.value(), msg))
            return true;
        if (!m_warnedMismatch.contains(msgName)) {
            m_warnedMismatch.insert(msgName);
            qWarning().noquote().nospace()
                    << "Replay: skipping broadcasts of '" << msgName
                    << "' - its logged payload does not match the current message definition "
                       "(log likely recorded with an older message format). Further notices for "
                       "this message are suppressed.";
        }
        return false;
    }

    /**
     * @brief Broadcasts a single telemetry frame over the Ivy bus in the exact wire
     * format produced by the previous OCaml player.
     *
     * @details Mirrors OCaml `run`/`loop`: a telemetry-class message emits
     * `replay<ac> <msg>` plus `time<ac> <t>` (t = the frame's own timestamp, six
     * decimals like OCaml `%f`), and a ground-class message emits `replay_ground <msg>`.
     * The two class checks are independent -- exactly like OCaml's two separate
     * `try/with` blocks -- so a message registered in both classes is forwarded twice.
     *
     * Centralising the send path keeps `onTimeout` lean and makes the engine ready for
     * live scrubbing: a seek handler can replay the frame(s) at the new slider position
     * simply by calling this method, with zero duplicated formatting. The multi-argument
     * `arg(a, b)` form also performs simultaneous substitution, so a payload that happens
     * to contain `%1`/`%2` can never be re-interpreted as a placeholder.
     */
    void sendFrame(const LogIndex &entry)
    {
        if (!m_bus)
            return;
        QString ac;
        QString msgName;
        QString msg;
        if (m_mappedData) {
            const char *data = reinterpret_cast<const char *>(m_mappedData + entry.offset);
            extractMsgData(data, entry.length, ac, msgName, msg);
        } else {
            m_dataFile.seek(entry.offset);
            QByteArray chunk = m_dataFile.read(entry.length);
            extractMsgData(chunk.constData(), chunk.length(), ac, msgName, msg);
        }
        if (msgName.isEmpty())
            return;

        if (m_telemetryMsgs.contains(msgName)) {
            // Robustness: never emit a frame whose integer fields don't match the current message
            // definition -- doing so would make OCaml/PprzLink consumers throw
            // Failure("int_of_string"). This guards both sequential playback and the live-scrub
            // snapshot, which calls sendFrame().
            if (frameIsBroadcastSafe(m_telIntChecks, msgName, msg)) {
                m_bus->send(QStringLiteral("replay%1 %2").arg(ac, msg));
                m_bus->send(
                        QStringLiteral("time%1 %2").arg(ac, QString::number(entry.time, 'f', 6)));
            }
        }
        if (m_groundMsgs.contains(msgName)) {
            if (frameIsBroadcastSafe(m_groundIntChecks, msgName, msg)) {
                m_bus->send(QStringLiteral("replay_ground %1").arg(msg));
            }
        }
    }

    /**
     * @brief Returns the index of the first frame at or after time `t`, clamped to valid bounds.
     */
    int indexForTime(double t) const
    {
        auto it = std::lower_bound(m_log.begin(),
                                   m_log.end(),
                                   t,
                                   [](const LogIndex &a, double tVal) { return a.time < tVal; });
        int idx = std::distance(m_log.begin(), it);
        if (idx >= m_log.size())
            idx = m_log.size() - 1;
        idx = std::max(idx, 0);
        return idx;
    }

    /**
     * @brief Lightweight stream-key extractor: returns "<ac> <msgName>" for a raw data line,
     * or an empty string if the line lacks the required leading fields.
     *
     * @details Deliberately mirrors the tokenizer in extractMsgData() but stops after the third
     * field and allocates only the short key (never the full payload), so building the per-stream
     * index at load time stays cheap even on multi-million-frame logs.
     */
    static QString streamKeyOf(const char *data, int lineLen)
    {
        int s1 = -1;
        int len1 = 0;
        int s2 = -1;
        int len2 = 0;
        int s3 = -1;
        int len3 = 0;
        for (int i = 0; i < lineLen; ++i) {
            const char c = data[i];
            if (c != ' ' && c != '\t' && c != '\r') {
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
        if (s3 != -1 && len3 == 0)
            len3 = lineLen - s3;
        if (s2 == -1 || len2 <= 0 || s3 == -1 || len3 <= 0)
            return {};
        return QString::fromUtf8(data + s2, len2) + QChar(' ') + QString::fromUtf8(data + s3, len3);
    }

    /**
     * @brief Builds the per-stream frame index used for O(streams x log n) scrub snapshots.
     *
     * @details Runs once per load, AFTER m_log has been time-sorted, so every stream's index
     * vector is itself chronologically ordered (indices into the sorted log are monotonic in
     * time). Keyed by "<ac> <msgName>", each entry lists the frames belonging to that stream,
     * letting broadcastSnapshot() binary-search the latest value of every signal at any instant.
     */
    void buildStreamIndex()
    {
        m_streamFrames.clear();
        QByteArray scratch;
        for (int i = 0; i < m_log.size(); ++i) {
            const LogIndex &e = m_log[i];
            const char *data;
            int len;
            if (m_mappedData) {
                data = reinterpret_cast<const char *>(m_mappedData + e.offset);
                len = e.length;
            } else {
                m_dataFile.seek(e.offset);
                scratch = m_dataFile.read(e.length);
                data = scratch.constData();
                len = static_cast<int>(scratch.size());
            }
            const QString key = streamKeyOf(data, len);
            if (!key.isEmpty())
                m_streamFrames[key].push_back(i);
        }
    }

    /**
     * @brief Broadcasts a coherent state snapshot at time `t`: the latest frame of every distinct
     * message stream at or before `t`, sent in chronological order via sendFrame().
     *
     * @details This is what makes scrubbing "live". For each stream a binary search finds the most
     * recent frame whose timestamp is <= t; that frame is (re)broadcast so the receiver shows the
     * exact state at the cursor regardless of travel direction. Total cost is O(streams x log n) --
     * independent of cursor position and tiny in practice (a few dozen streams), which keeps
     * scrubbing responsive even on gigabyte logs. The scratch buffer is reused to stay
     * allocation-free in steady state.
     */
    void broadcastSnapshot(double t)
    {
        if (!m_bus || m_streamFrames.isEmpty())
            return;
        m_snapshotScratch.clear();
        m_snapshotScratch.reserve(m_streamFrames.size());
        for (auto it = m_streamFrames.cbegin(); it != m_streamFrames.cend(); ++it) {
            const QVector<int> &v = it.value();
            // Predecessor of the first frame whose time exceeds t == latest frame with time <= t.
            int lo = 0;
            int hi = static_cast<int>(v.size());
            while (lo < hi) {
                const int mid = (lo + hi) >> 1;
                if (m_log[v[mid]].time <= t)
                    lo = mid + 1;
                else
                    hi = mid;
            }
            if (lo > 0)
                m_snapshotScratch.push_back(v[lo - 1]);
        }
        std::sort(m_snapshotScratch.begin(), m_snapshotScratch.end());
        for (int fi : std::as_const(m_snapshotScratch))
            sendFrame(m_log[fi]);
    }

    /**
     * @brief Maps physical configuration settings from target log environments into OS paths.
     *
     * @details Extracted fully representing logic formerly processed by `Ocaml ExtXml` nodes.
     * Generates native `airframe`, `radio`, `flight_plan` directories inside
     * `$PAPARAZZI_HOME/var/replay` enabling external tool execution explicitly alongside the
     * dataset replay natively.
     */
    static void storeConf(const QDomElement &root, const QSet<QString> &acs)
    {
        QString pprzHome = paparazziHome();

        QString replayDir = pprzHome + QStringLiteral("/var/replay");
        QDir().mkpath(replayDir + QStringLiteral("/conf"));
        QDir().mkpath(replayDir + QStringLiteral("/var/aircrafts"));

        QDomNodeList confNodes = root.elementsByTagName(QStringLiteral("conf"));
        if (confNodes.isEmpty())
            return;
        QDomElement confEl = confNodes.at(0).toElement();

        QDomDocument outDoc;
        QDomElement outConf = outDoc.createElement(QStringLiteral("conf"));
        outDoc.appendChild(outConf);

        QDomNode child = confEl.firstChild();
        while (!child.isNull()) {
            if (child.isElement() && child.nodeName() == QLatin1String("aircraft")) {
                QDomElement acEl = child.toElement();
                QString acId = acEl.attribute(QStringLiteral("ac_id"));

                // Process solely AC configurations actually encountered internally matching logged
                // identifiers
                if (acs.contains(acId)) {
                    QString acName = acEl.attribute(QStringLiteral("name"));
                    QString acDirStr = replayDir + QStringLiteral("/var/aircrafts/") + acName;
                    QDir acDir(acDirStr);
                    acDir.mkpath(QStringLiteral("."));
                    acDir.mkpath(QStringLiteral("conf"));

                    auto writeXmlFile = [](const QString &path, const QDomElement &el) {
                        QFileInfo fi(path);
                        fi.absoluteDir().mkpath(QStringLiteral(
                                ".")); // Force dependency folders into existence natively!
                        QFile f(path);
                        if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
                            QTextStream out(&f);
                            el.save(out, 2);
                        } else {
                            qWarning() << "Failed to dump log asset to:" << path;
                        }
                    };

                    auto extractChild = [&](const QString &tag) {
                        QDomNodeList list = acEl.elementsByTagName(tag);
                        if (!list.isEmpty()) {
                            QDomElement el = list.at(0).toElement();
                            writeXmlFile(replayDir + QStringLiteral("/conf/") + acEl.attribute(tag),
                                         el);
                            writeXmlFile(acDirStr + QStringLiteral("/conf/") + acEl.attribute(tag),
                                         el);
                        }
                    };

                    extractChild(QStringLiteral("airframe"));
                    extractChild(QStringLiteral("radio"));

                    QDomNodeList genSet
                            = acEl.elementsByTagName(QStringLiteral("generated_settings"));
                    if (!genSet.isEmpty()) {
                        QDomElement settingsXml = genSet.at(0).toElement();
                        writeXmlFile(acDirStr + QStringLiteral("/settings.xml"), settingsXml);
                        writeXmlFile(replayDir + QStringLiteral("/settings.xml"), settingsXml);
                    } else {
                        qWarning() << "Replay: no settings for display natively bundled in log.";
                        QDomElement dummy = outDoc.createElement(QStringLiteral("settings"));
                        writeXmlFile(acDirStr + QStringLiteral("/settings.xml"), dummy);
                        writeXmlFile(replayDir + QStringLiteral("/settings.xml"), dummy);
                    }

                    bool orig_fp = !acEl.elementsByTagName(QStringLiteral("flight_plan")).isEmpty();
                    if (orig_fp) {
                        // Automatically compile target Flight Plan out outputs using PAPARAZZI
                        // tooling natively.
                        extractChild(QStringLiteral("flight_plan"));
                        QString fpName = acEl.attribute(QStringLiteral("flight_plan"));
                        QString dumpFpProg = paparazziSrc()
                                + QStringLiteral("/sw/tools/generators/dump_flight_plan.out");
                        // OCaml feeded the aircraft-local copy (the return value of `w`) to
                        // dump_flight_plan; both copies are byte-identical, but using the
                        // ac_dir path keeps include-resolution depth identical to OCaml.
                        QString fpath = acDirStr + QStringLiteral("/conf/") + fpName;
                        QString dumpPath = acDirStr + QStringLiteral("/flight_plan.xml");
                        QProcess::execute(dumpFpProg, {fpath, dumpPath});
                    } else {
                        // Support dump blocks universally generated via active legacy logging
                        // routines.
                        QDomNodeList dumps = acEl.elementsByTagName(QStringLiteral("dump"));
                        if (!dumps.isEmpty()) {
                            writeXmlFile(acDirStr + QStringLiteral("/flight_plan.xml"),
                                         dumps.at(0).toElement());
                        }
                    }
                    // OCaml emits `Xml.Element ("aircraft", Xml.attribs x, [])` -- the
                    // aircraft node carries its attributes but NO children in conf.xml
                    // (the expanded airframe/radio/fp subtrees live under var/aircrafts).
                    // A shallow clone reproduces that exactly; a deep clone would leak the
                    // full log payload into conf.xml and break downstream conf consumers.
                    outConf.appendChild(acEl.cloneNode(false));
                }
            } else {
                outConf.appendChild(child.cloneNode(true));
            }
            child = child.nextSibling();
        }

        QFile outF(replayDir + QStringLiteral("/conf/conf.xml"));
        if (outF.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream outT(&outF);
            outDoc.save(outT, 2);
        }
    }

    /**
     * @brief Integrates Paparazzi's native XML protocol Dictionary definitions mapping metadata.
     */
    static void storeMessages(const QDomElement &root)
    {
        QString pprzHome = paparazziHome();
        QString replayDir = pprzHome + QStringLiteral("/var/replay");
        QDir().mkpath(replayDir + QStringLiteral("/var"));

        QDomNodeList protoNodes = root.elementsByTagName(QStringLiteral("protocol"));
        if (!protoNodes.isEmpty()) {
            QFile protoFile(replayDir + QStringLiteral("/var/messages.xml"));
            if (protoFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QTextStream outT(&protoFile);
                QString protoStr;
                QTextStream tmp(&protoStr);
                protoNodes.at(0).save(tmp, 2);
                tmp.flush(); // IMPORTANT: flush before string operations!

                // GUARANTEE: Legacy logger outputs explicitly uppercase attributes (NAME, ID, TYPE)
                // which violates case-sensitive PprzLinkCPP dictionary parsers. Force normalize
                // bindings!
                protoStr.replace(QRegularExpression(QStringLiteral("\\bNAME=")),
                                 QStringLiteral("name="));
                protoStr.replace(QRegularExpression(QStringLiteral("\\bID=")),
                                 QStringLiteral("id="));
                protoStr.replace(QRegularExpression(QStringLiteral("\\bTYPE=")),
                                 QStringLiteral("type="));
                protoStr.replace(QRegularExpression(QStringLiteral("\\bUNIT=")),
                                 QStringLiteral("unit="));
                protoStr.replace(QRegularExpression(QStringLiteral("\\bALT_UNIT_COEF=")),
                                 QStringLiteral("alt_unit_coef="));
                protoStr.replace(QRegularExpression(QStringLiteral("\\bVALUES=")),
                                 QStringLiteral("values="));

                outT << "<?xml version=\"1.0\"?>\n";
                // ---------------------------------------------------------------------------
                // WORKAROUND: deliberately DO NOT emit `<!DOCTYPE protocol SYSTEM "messages.dtd">`.
                //
                // This messages.xml lands in <PAPARAZZI_HOME>/var/replay/var/ and is later read by
                // other Paparazzi tools (GCS / server) that parse with OCaml's xml-light. xml-light
                // resolves a SYSTEM DTD relative to the XML file, so a DOCTYPE pointing at
                // "messages.dtd" makes it look for <...>/var/replay/var/messages.dtd -- a file we
                // never generate -- and it aborts with:
                //     Xml_light_errors.File_not_found(".../var/replay/var/messages.dtd")
                // The original OCaml store_messages (Xml.to_string_fmt) emitted no DOCTYPE either,
                // so omitting it also restores behavioral parity. The C++ PprzLinkCPP reader used
                // in initDictionary() ignores DOCTYPE, so this is safe for our own parsing too.
                //
                // POSSIBLE IMPROVEMENT (proper fix vs. this workaround): if DTD validation is
                // wanted during development, copy the canonical DTD
                // (paparazziSrc() + "/conf/messages.dtd") into this same directory alongside the
                // generated messages.xml, then re-enable the DOCTYPE line below -- ideally gated
                // behind a dev/debug flag so production replay stays dependency-free:
                //     // outT << "<!DOCTYPE protocol SYSTEM \"messages.dtd\">\n";
                // ---------------------------------------------------------------------------
                outT << protoStr;
            }
        }
    }

    /**
     * @brief Initializes dictionary classification indices parsing variables dynamically reliably.
     */
    void initDictionary()
    {
        QString pprzHome = paparazziHome();

        // --- Replay dictionary: classifies each logged message as telemetry vs ground ------------
        // This is the message set the log was *recorded* with (var/replay/var/messages.xml) -- the
        // .log embeds it verbatim and storeMessages() just wrote it back out. It is used only to
        // route each frame to the right replay channel, mirroring play_core.ml which tests
        // Tm_Pprz/Ground_Pprz.message_of_name. It deliberately does NOT drive value checking: it
        // matches the log by construction and so can never reveal a consumer-side parse problem.
        // Parsing is plain QtXml -- no message library is needed to re-emit logged lines on Ivy.
        {
            QDomDocument doc;
            const QDomElement proto
                    = openProtocol(pprzHome + QStringLiteral("/var/replay/var/messages.xml"), doc);
            if (!proto.isNull()) {
                collectClassMessageNames(proto, QStringLiteral("ground"), m_groundMsgs);
                for (const QString &tc : {QStringLiteral("telemetry"),
                                          QStringLiteral("telemetry_ap"),
                                          QStringLiteral("telemetry_fbw")}) {
                    collectClassMessageNames(proto, tc, m_telemetryMsgs);
                }
                m_timeScaleIdx = fieldIndex(
                        proto, QStringLiteral("WORLD_ENV"), QStringLiteral("time_scale"));
            } else {
                qWarning() << "Replay: routing dictionary unavailable "
                              "(var/replay/var/messages.xml); no messages can be routed for "
                              "broadcast.";
            }
        }

        // --- Consumer dictionary: decides whether a payload is *parseable* by receivers ----------
        // Receivers on the Ivy bus (GCS, server, ...) decode replay messages with the CURRENT
        // message set, not the log's old set. When a message's field layout changed between the two
        // -- e.g. IMU_GYRO/IMU_ACCEL/IMU_MAG gained a leading uint8 'id' field -- an old log
        // supplies a float where the consumer now expects an integer, so the consumer's
        // int_of_string raises Failure("int_of_string"). We therefore build the integer-field
        // checks from THIS dictionary; sendFrame() then skips any frame whose integer fields would
        // not parse under it. If the file is unavailable the checks stay empty and we fall back to
        // the previous (unchecked) behaviour rather than refusing to replay.
        //
        // The dictionary defaults to the live build's $PAPARAZZI_HOME/var/messages.xml (what the
        // standard agents use), but --consumer-messages <file> overrides it so a user can validate
        // against the exact set their GCS/server runs -- e.g. replaying into a non-default or
        // remote configuration. An explicit-but-missing path is reported loudly (below) because a
        // silent fallback to no validation would hide a user mistake; the default path stays quiet
        // so a bare install without var/messages.xml simply replays unchecked, exactly as before.
        const bool explicitConsumer = !m_consumerMessagesPath.isEmpty();
        const QString consumerPath = explicitConsumer
                ? m_consumerMessagesPath
                : (pprzHome + QStringLiteral("/var/messages.xml"));
        if (explicitConsumer && !QFile::exists(consumerPath)) {
            qWarning().noquote() << "Replay: --consumer-messages file not found, payload "
                                    "int-validation disabled:"
                                 << consumerPath;
        }
        {
            QDomDocument doc;
            const QDomElement proto = openProtocol(consumerPath, doc);
            if (!proto.isNull()) {
                // Route each class's messages to the matching check table, mirroring the original
                // getMsgsForClass(telemetry|telemetry_ap|telemetry_fbw) and
                // getMsgsForClass(ground).
                for (QDomElement cls = proto.firstChildElement(QStringLiteral("msg_class"));
                     !cls.isNull();
                     cls = cls.nextSiblingElement(QStringLiteral("msg_class"))) {
                    const QString cn = cls.attribute(QStringLiteral("name"));
                    QHash<QString, QVector<IntFieldCheck>> *target = nullptr;
                    if (cn == QLatin1String("ground")) {
                        target = &m_groundIntChecks;
                    } else if (cn == QLatin1String("telemetry")
                               || cn == QLatin1String("telemetry_ap")
                               || cn == QLatin1String("telemetry_fbw")) {
                        target = &m_telIntChecks;
                    }
                    if (!target)
                        continue;
                    for (QDomElement m = cls.firstChildElement(QStringLiteral("message"));
                         !m.isNull();
                         m = m.nextSiblingElement(QStringLiteral("message"))) {
                        buildIntChecks(m, *target);
                    }
                }
            }
            // Absent/unreadable consumer dictionary -> checks stay empty -> replay stays unchecked.
        }
    }

    double m_speed {1.0}; ///< Engine Multiplier tracking relative simulation speeds.
    int m_currentIndex {0}; ///< Target line alignment pointing specifically towards next execution
                            ///< blocks.
    bool m_isPlaying {false}; ///< Standard Operational State explicitly pausing functional triggers
                              ///< natively cleanly.
    double m_virtualTime {0.0}; ///< Absolute numerical representation binding structural variables
                                ///< tracking chronological variables precisely explicitly.
    int m_timeScaleIdx {
            -1}; ///< Operational boundary dictating structural scaling mapping external
                 ///< elements reliably dynamically uniquely normally fully cleanly safely.
    bool m_noGui = false; ///< Legacy toggle explicitly masking generic components reproducing
                          ///< play-nox variables identically implicitly.
    QString m_consumerMessagesPath; ///< Optional explicit consumer messages.xml
                                    ///< (--consumer-messages); empty => default
                                    ///< $PAPARAZZI_HOME/var/messages.xml.

    QTimer *m_tickTimer; ///< Precision component bounding display metrics dynamically safely
                         ///< mapping limits natively normally accurately inherently dynamically.
    QElapsedTimer
            m_elapsed; ///< Clock element implicitly driving tracking strings purely identically
                       ///< driving loops flawlessly organically normally uniquely efficiently.

    QString m_xmlFile; ///< Functional generic origin boundary strictly uniquely correctly
                       ///< generating tracks linearly efficiently.
    QVector<LogIndex> m_log; ///< Operational boundary structure storing mapped locations accurately
                             ///< linearly uniquely organically normally efficiently.
    QHash<QString, QVector<int>>
            m_streamFrames; ///< Per-stream ("<ac> <msgName>") frame indices into the sorted log;
                            ///< powers O(log n) scrub snapshots.
    QVector<int> m_snapshotScratch; ///< Reused buffer for snapshot frame collection (steady-state
                                    ///< zero-allocation).
    QFile m_dataFile; ///< Physical boundary pointer limiting load conditions functionally tracking
                      ///< loops functionally normally implicitly implicitly natively naturally
                      ///< efficiently.
    uchar *m_mappedData {
            nullptr}; ///< Explicit memory boundary zero-allocation target mapping securely.

    IvyQt *m_bus {nullptr}; ///< Network broadcast pipeline seamlessly handling external payloads.
    QString m_ivyBusArg; ///< Defined Ivy domain structural limits.

    QSet<QString>
            m_groundMsgs; ///< Classification index cleanly isolating transmission vectors natively.
    QSet<QString>
            m_telemetryMsgs; ///< Telemetry class indicator map generating strict array limitations.
    QHash<QString, QVector<IntFieldCheck>>
            m_telIntChecks; ///< Per telemetry msg: integer fields to validate before broadcast.
    QHash<QString, QVector<IntFieldCheck>>
            m_groundIntChecks; ///< Per ground msg: integer fields to validate before broadcast.
    QSet<QString> m_warnedMismatch; ///< Message names already warned about (one notice each).
};

/**
 * @class PlayWindow
 * @brief GUI Controller coordinating Front-End operational requirements directly pushing boundaries
 * internally cleanly identically safely natively.
 */
class PlayWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit PlayWindow(PlayCore *core, QWidget *parent = nullptr)
        : QMainWindow(parent)
        , m_core(core)
    {
        setAttribute(Qt::WA_DeleteOnClose);
        setWindowTitle(QStringLiteral("Paparazzi Replay"));
        resize(480, 100);

        // Mimic original OCaml GTK Menu ("File" -> "Open Log", "Play", "Stop", "Quit")
        QMenu *fileMenu = menuBar()->addMenu(tr("&File"));

        QAction *actionOpen = fileMenu->addAction(tr("Open Log"));
        actionOpen->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_O));

        // Single Play/Pause action: text + behaviour toggle, kept congruent with the
        // Play/Pause button via onPlayStateChanged(). Starts life as "Play".
        m_actionPlayPause = fileMenu->addAction(tr("Play"));
        m_actionPlayPause->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_X)); // OCaml used _X

        // Stop = rewind to the very start of the log, then wait (paused) for the user to Play.
        m_actionStop = fileMenu->addAction(tr("Stop"));
        m_actionStop->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_S)); // OCaml used _S

        fileMenu->addSeparator();

        QAction *actionQuit = fileMenu->addAction(tr("Quit"));
        actionQuit->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_Q));

        connect(actionOpen, &QAction::triggered, this, &PlayWindow::onOpen);
        connect(m_actionPlayPause, &QAction::triggered, this, &PlayWindow::onPlayPause);
        connect(m_actionStop, &QAction::triggered, this, &PlayWindow::onStop);
        connect(actionQuit, &QAction::triggered, qApp, &QApplication::quit);

        QWidget *central = new QWidget(this);
        QVBoxLayout *vlayout = new QVBoxLayout(central);

        QHBoxLayout *tools = new QHBoxLayout();
        const QStyle *st = style();

        // [ Back ]  fast-backward by the fixed compile-time step. Carries text + icon (not
        // icon-only) so the glyph is always drawn -- including while disabled before a log is
        // loaded.
        m_btnBackward = new QPushButton(st->standardIcon(QStyle::SP_MediaSeekBackward), tr("Back"));
        m_btnBackward->setToolTip(QStringLiteral("Jump backward %1 s").arg(PLAY_SEEK_STEP_SECONDS));

        // [ Play / Pause ]  fused toggle; icon + text track the engine state (see
        // onPlayStateChanged).
        m_btnPlayPause = new QPushButton(st->standardIcon(QStyle::SP_MediaPlay), tr("Play"));
        m_btnPlayPause->setToolTip(QStringLiteral("Play / Pause"));

        // [ Fwd ]  fast-forward by the fixed compile-time step. Text + icon, same as Back.
        m_btnForward = new QPushButton(st->standardIcon(QStyle::SP_MediaSeekForward), tr("Fwd"));
        m_btnForward->setToolTip(QStringLiteral("Jump forward %1 s").arg(PLAY_SEEK_STEP_SECONDS));

        // Force the three transport buttons to share one height (the icon-only Back/Fwd used to
        // render shorter than the text Play button) and a fixed vertical policy so the row stays
        // visually uniform and never stretches.
        const int transportBtnH = m_btnPlayPause->sizeHint().height();
        m_btnBackward->setMinimumHeight(transportBtnH);
        m_btnPlayPause->setMinimumHeight(transportBtnH);
        m_btnForward->setMinimumHeight(transportBtnH);
        m_btnBackward->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        m_btnPlayPause->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        m_btnForward->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

        m_speedBox = new QDoubleSpinBox();
        m_speedBox->setToolTip(QStringLiteral("Playback Speed Multiplier"));
        m_speedBox->setRange(0.01, 100.0);
        m_speedBox->setSingleStep(0.5);
        m_speedBox->setValue(1.0);
        m_speedBox->setPrefix(QStringLiteral("x "));

        m_timeLabel = new QLabel(QStringLiteral("00:00 / 00:00"));
        m_timeLabel->setAlignment(Qt::AlignCenter);
        m_timeLabel->setMinimumWidth(100);

        tools->addWidget(m_btnBackward);
        tools->addWidget(m_btnPlayPause);
        tools->addWidget(m_btnForward);
        tools->addWidget(m_speedBox);
        tools->addWidget(m_timeLabel);
        vlayout->addLayout(tools);

        // Transport controls stay disabled until a log is successfully loaded.
        setTransportEnabled(false);

        m_slider = new QSlider(Qt::Horizontal);
        m_slider->setPageStep(500);
        m_slider->setEnabled(false); // becomes interactive once a log is loaded
        m_slider->setToolTip(QStringLiteral("Drag to scrub backward/forward through the log"));
        vlayout->insertWidget(0, m_slider); // pin the timeline slider to the TOP of the form
        vlayout->addStretch(1); // absorb any spare space at the BOTTOM so the slider +
                                // button row stay flush under the menu (top-aligned)

        // Coalescing timer that bounds how often the cheap-but-bus-heavy scrub snapshot is sent.
        m_scrubThrottle = new QTimer(this);
        m_scrubThrottle->setSingleShot(true);
        m_scrubThrottle->setInterval(kScrubThrottleMs);

        // End-of-scrub detector: fires once the user has stopped moving the slider for a moment.
        // This is what makes auto-resume robust for interaction modes that never emit
        // sliderReleased (groove clicks, keyboard, mouse wheel).
        m_scrubIdle = new QTimer(this);
        m_scrubIdle->setSingleShot(true);
        m_scrubIdle->setInterval(kScrubIdleMs);

        setCentralWidget(central);

        connect(m_btnPlayPause, &QPushButton::clicked, this, &PlayWindow::onPlayPause);
        connect(m_btnForward, &QPushButton::clicked, this, &PlayWindow::onSeekForward);
        connect(m_btnBackward, &QPushButton::clicked, this, &PlayWindow::onSeekBackward);
        connect(m_speedBox,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                m_core,
                &PlayCore::setSpeed);
        connect(m_core, &PlayCore::speedChangedByNetwork, m_speedBox, &QDoubleSpinBox::setValue);
        // Single source of truth for the Play/Pause UI (button + menu), regardless of trigger.
        connect(m_core, &PlayCore::stateChanged, this, &PlayWindow::onPlayStateChanged);

        // Core Interaction Guard: Programmatic `setValue` commands trigger slider signals causing
        // cyclic jump commands. Uniquely binds user inputs exclusively protecting processing
        // operations robustly.
        connect(m_slider, &QAbstractSlider::valueChanged, this, &PlayWindow::onSliderValueChanged);

        // --- Live scrubbing wiring --------------------------------------------------------
        // A scrub is detected from valueChanged -- the ONLY signal every interaction mode emits
        // (handle drag, groove click, keyboard, wheel). Playback's own slider updates are
        // blockSignals-guarded upstream, so they never reach the scrub path. beginScrub() pauses
        // playback and remembers whether it was running; finishScrub() resumes it.
        //
        // End-of-scrub is detected by whichever of these fires first:
        //   * sliderReleased -> instant resume when the user lets go of the handle.
        //   * m_scrubIdle    -> resume ~150 ms after the LAST value change; the robust fallback
        //                       for groove clicks / keyboard / wheel, where sliderReleased never
        //                       fires (these were the cases that left playback stuck paused).
        // Both funnel through the idempotent finishScrub(), so a double trigger is harmless.
        connect(m_slider, &QSlider::sliderPressed, this, &PlayWindow::beginScrub);
        connect(m_slider, &QSlider::sliderReleased, this, &PlayWindow::finishScrub);
        connect(m_scrubIdle, &QTimer::timeout, this, &PlayWindow::finishScrub);

        // The throttle's trailing tick coalesces rapid drag updates so the Ivy bus is never
        // flooded.
        connect(m_scrubThrottle, &QTimer::timeout, this, [this] {
            if (m_scrubPendingFlush) {
                m_scrubPendingFlush = false;
                m_core->scrubTo(m_pendingScrubT);
                m_scrubThrottle->start(); // keep coalescing while the drag continues
            }
        });

        connect(m_core, &PlayCore::logLoaded, this, &PlayWindow::onLogLoaded);
        connect(m_core, &PlayCore::timeUpdated, this, &PlayWindow::onTimeUpdated);

        // Fit the form to its content now (menu + slider + button row) and pick a sensible default
        // width; the height is then frozen on first show (see showEvent).
        adjustSize();
        resize(480, height());
    }

protected:
    /**
     * @brief Freezes the window height to its natural content height on first show, so the form
     * fits the slider and button row exactly and can no longer be stretched vertically, while the
     * width remains freely resizable.
     */
    void showEvent(QShowEvent *ev) override
    {
        QMainWindow::showEvent(ev);
        if (!m_heightLocked) {
            m_heightLocked = true;
            setFixedHeight(sizeHint().height());
        }
    }

private:
    void onOpen()
    {
        m_core->stop();
        QString fileName;
        {
            StderrBlocker blocker;
            const QString defaultLogExtPath = QStringLiteral("var/logs");
            // Primary default mirrors OCaml `Log_file.logs_dir` = $PAPARAZZI_HOME/var/logs
            // (with the same $HOME/paparazzi fallback baked into paparazziHome()).
            QString logDir = QDir(paparazziHome()).filePath(defaultLogExtPath);
            if (!QDir(logDir).exists()) {
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
                    QStringLiteral("Log Files (*.log *.xml);;All Files (*)"));
        }
        if (!fileName.isEmpty()) {
            if (m_core->loadLog(fileName)) {
                setWindowTitle(QStringLiteral("Replay: ") + QFileInfo(fileName).fileName());
            }
        }
    }

    void onLogLoaded(double minT, double maxT)
    {
        m_slider->setRange(0, kSliderSteps);
        m_minT = minT;
        m_maxT = maxT;
        m_currentT = minT;
        m_slider->setEnabled(true); // log present -> scrubbing available
        setTransportEnabled(true); // Play/Pause + fast-seek become usable
        m_slider->blockSignals(true);
        m_slider->setValue(0);
        m_slider->blockSignals(false);
        updateLabel(minT);
    }

    void onTimeUpdated(double currentT)
    {
        m_currentT = currentT; // authoritative playback position; anchors seek-by-delta
        if (!m_slider->isSliderDown() && m_maxT > m_minT) {
            double frac = (currentT - m_minT) / (m_maxT - m_minT);
            m_slider->blockSignals(true);
            m_slider->setValue(qBound(0, static_cast<int>(frac * kSliderSteps), kSliderSteps));
            m_slider->blockSignals(false);
            updateLabel(currentT);
        }
        updateSeekButtons(); // Back/Forward track the head: disabled at the very start/end
    }

    void onSliderValueChanged(int val)
    {
        if (m_maxT <= m_minT)
            return;
        // Robust begin: every user-driven value change marks an active scrub (and pauses playback
        // the first time). Programmatic updates from playback are blockSignals-guarded upstream,
        // so they never reach this slot.
        beginScrub();
        const double t = m_minT + ((val / static_cast<double>(kSliderSteps)) * (m_maxT - m_minT));
        m_currentT = t; // keep seek-by-delta anchored to the latest scrub position
        updateLabel(t); // instant visual feedback, independent of broadcast throttling
        updateSeekButtons(); // live-enable/disable Back/Forward as the slider nears the ends
        m_pendingScrubT = t;
        if (m_scrubThrottle->isActive()) {
            m_scrubPendingFlush
                    = true; // coalesce; the trailing timer tick broadcasts the latest position
        } else {
            m_core->scrubTo(t); // leading edge: broadcast immediately so scrubbing feels responsive
            m_scrubThrottle->start(); // open the throttle window
        }
        m_scrubIdle->start(); // (re)arm end-of-scrub detection; resume fires once movement stops
    }

    /**
     * @brief Marks the start of a scrub gesture: records whether playback was active, then pauses
     * it. Idempotent -- safe to call on sliderPressed and on every subsequent value change.
     */
    void beginScrub()
    {
        if (m_scrubbing)
            return;
        m_scrubbing = true;
        m_wasPlayingBeforeScrub = m_core->isPlaying();
        if (m_wasPlayingBeforeScrub)
            m_core->stop();
    }

    /**
     * @brief Ends a scrub gesture: flushes the exact final position and resumes playback if it was
     * running when the scrub began. Idempotent -- whichever of sliderReleased / m_scrubIdle fires
     * first wins; the other becomes a no-op. While the handle is still physically held
     * (isSliderDown), resume is deferred to sliderReleased so the idle timer can't resume mid-drag.
     */
    void finishScrub()
    {
        if (!m_scrubbing)
            return;
        if (m_slider->isSliderDown())
            return; // handle still held: let sliderReleased finish the gesture
        m_scrubbing = false;
        m_scrubIdle->stop();
        m_scrubThrottle->stop();
        if (m_scrubPendingFlush) {
            m_scrubPendingFlush = false;
            m_core->scrubTo(m_pendingScrubT); // guarantee the exact final position is broadcast
        }
        if (m_wasPlayingBeforeScrub) {
            m_wasPlayingBeforeScrub = false;
            m_core->play(); // seamless resume from the scrubbed position
        }
    }

    /**
     * @brief Play/Pause toggle shared by the transport button and the menu action.
     * Pause keeps the current position (it is the old "stop" semantics = timer halt); Play
     * resumes from there (PlayCore::play() auto-rewinds only when the head sits at end-of-log).
     */
    void onPlayPause()
    {
        if (m_core->isPlaying())
            m_core->stop(); // Pause: halt the timer, keep position
        else
            m_core->play(); // Play : resume from the current position
    }

    /**
     * @brief Stop: rewind to the very start of the log and wait, paused, for the user to Play.
     * Deliberately distinct from Pause -- this one resets the playback head to the beginning.
     */
    void onStop()
    {
        m_core->stop(); // ensure paused first (so scrubTo performs no clock restart)
        if (m_maxT > m_minT) {
            m_currentT = m_minT;
            m_core->scrubTo(m_minT); // seek head to start + broadcast the initial-state snapshot
        }
    }

    /// @brief Fast-forward by the fixed compile-time step (PLAY_SEEK_STEP_SECONDS).
    void onSeekForward() { seekByDelta(+PLAY_SEEK_STEP_SECONDS); }
    /// @brief Fast-backward by the fixed compile-time step (PLAY_SEEK_STEP_SECONDS).
    void onSeekBackward() { seekByDelta(-PLAY_SEEK_STEP_SECONDS); }

    /**
     * @brief Keeps the Play/Pause button AND menu action congruent with the engine state, no
     * matter what changed it (button, menu, end-of-log auto-stop, or scrub pause/resume).
     */
    void onPlayStateChanged(bool playing)
    {
        const QStyle *st = style();
        const QIcon icon = st->standardIcon(playing ? QStyle::SP_MediaPause : QStyle::SP_MediaPlay);
        const QString text = playing ? tr("Pause") : tr("Play");
        m_btnPlayPause->setIcon(icon);
        m_btnPlayPause->setText(text);
        if (m_actionPlayPause)
            m_actionPlayPause->setText(text);
    }

    /**
     * @brief Seeks the playback head by `delta` seconds, clamped to the log bounds. Works whether
     * playing or paused: PlayCore::scrubTo() restarts the playback clock when active (so playback
     * continues seamlessly from the new spot) and always broadcasts a coherent state snapshot.
     */
    void seekByDelta(double delta)
    {
        if (m_maxT <= m_minT)
            return; // empty / single-frame log -> nothing to seek
        const double t = qBound(m_minT, m_currentT + delta, m_maxT);
        m_currentT = t;
        m_core->scrubTo(t);
    }

    /**
     * @brief Enables/disables every transport control at once (button trio + menu actions).
     * Held false until a log loads, true afterwards -- so the UI can never drive an empty engine.
     */
    void setTransportEnabled(bool on)
    {
        // Play/Pause and Stop (button + menu twins) follow the log-loaded state directly.
        if (m_btnPlayPause)
            m_btnPlayPause->setEnabled(on);
        if (m_actionPlayPause)
            m_actionPlayPause->setEnabled(on);
        if (m_actionStop)
            m_actionStop->setEnabled(on);
        // Back/Forward additionally depend on the head position (disabled at the very ends), so
        // when enabling we defer to updateSeekButtons(); when disabling we force them both off.
        if (on) {
            updateSeekButtons();
        } else {
            if (m_btnBackward)
                m_btnBackward->setEnabled(false);
            if (m_btnForward)
                m_btnForward->setEnabled(false);
        }
    }

    /**
     * @brief Enables Back/Forward only when a log is loaded AND the head is away from that end:
     * Back is disabled at the very start, Forward at the very end. Position is read from m_currentT
     * (the authoritative head) with a one-slider-step epsilon, so truncation in the time->slider
     * mapping can never leave a button wrongly enabled right at an extreme.
     */
    void updateSeekButtons()
    {
        const double range = m_maxT - m_minT;
        const bool haveLog = range > 0.0;
        const double eps = haveLog ? range / kSliderSteps : 0.0;
        const bool atStart = !haveLog || m_currentT <= m_minT + eps;
        const bool atEnd = !haveLog || m_currentT >= m_maxT - eps;
        if (m_btnBackward)
            m_btnBackward->setEnabled(haveLog && !atStart);
        if (m_btnForward)
            m_btnForward->setEnabled(haveLog && !atEnd);
    }

    /**
     * @brief Renders the playback head and total duration as human-readable clocks,
     * measured relative to the log's start (so a freshly loaded log reads 00:00) and
     * widening automatically to H:MM:SS for logs longer than one hour.
     */
    void updateLabel(double currentT)
    {
        if (!std::isfinite(currentT) || !std::isfinite(m_maxT))
            return;
        const double elapsed = std::max(0.0, currentT - m_minT);
        const double total = std::max(0.0, m_maxT - m_minT);
        m_timeLabel->setText(formatClock(elapsed) + QStringLiteral(" / ") + formatClock(total));
    }

    /**
     * @brief Formats a non-negative second count as MM:SS, or H:MM:SS when >= 1 hour.
     */
    static QString formatClock(double seconds)
    {
        const int totalSecs = static_cast<int>(seconds);
        const int h = totalSecs / 3600;
        const int m = (totalSecs % 3600) / 60;
        const int s = totalSecs % 60;
        if (h > 0) {
            return QStringLiteral("%1:%2:%3")
                    .arg(h)
                    .arg(m, 2, 10, QChar('0'))
                    .arg(s, 2, 10, QChar('0'));
        }
        return QStringLiteral("%1:%2").arg(m, 2, 10, QChar('0')).arg(s, 2, 10, QChar('0'));
    }

    PlayCore *m_core; ///< Pointer coordinating GUI interactions dynamically bounded to internal
                      ///< engines.
    QSlider *m_slider; ///< User structural UI input bounds explicitly tracking track position.
    QDoubleSpinBox *m_speedBox; ///< Visual speed multiplier tracking limits intuitively.
    QLabel *m_timeLabel; ///< Human readable clock supporting visual string outputs seamlessly.
    QPushButton *m_btnPlayPause
            = nullptr; ///< Fused Play/Pause toggle (icon + text track engine state).
    QPushButton *m_btnForward = nullptr; ///< Fast-forward: jumps +PLAY_SEEK_STEP_SECONDS.
    QPushButton *m_btnBackward = nullptr; ///< Fast-backward: jumps -PLAY_SEEK_STEP_SECONDS.
    QAction *m_actionPlayPause = nullptr; ///< Menu twin of the Play/Pause button (kept congruent).
    QAction *m_actionStop = nullptr; ///< Menu Stop = rewind to start, then wait paused.
    double m_currentT = 0.0; ///< Latest playback position; anchor for seek-by-delta.
    static constexpr int kSliderSteps
            = 10000; ///< Slider granularity; higher = finer live-scrubbing resolution.
    static constexpr int kScrubThrottleMs
            = 33; ///< Min interval between scrub snapshot broadcasts (~30 Hz) to keep the bus calm.
    QTimer *m_scrubThrottle
            = nullptr; ///< Coalesces rapid drag updates so the Ivy bus is never flooded.
    static constexpr int kScrubIdleMs
            = 150; ///< Idle gap after the last slider move that marks end-of-scrub.
    QTimer *m_scrubIdle
            = nullptr; ///< Fires kScrubIdleMs after the last move; robust end-of-scrub resume.
    bool m_scrubbing
            = false; ///< True between begin/finish of a scrub gesture (any interaction mode).
    double m_pendingScrubT = 0.0; ///< Latest scrub target awaiting a throttled broadcast.
    bool m_scrubPendingFlush
            = false; ///< True when a coalesced scrub position still needs to be sent.
    bool m_wasPlayingBeforeScrub
            = false; ///< Remembers play state at scrub start so playback auto-resumes on end.
    double m_minT = 0,
           m_maxT
            = 0; ///< Explicitly cached variable boundaries logically limiting dynamic parameters.
    bool m_heightLocked
            = false; ///< True once the window height has been frozen to its content (first show).
};

/**
 * @brief Point of initialization setting Desktop hooks globally.
 */
int main(int argc, char *argv[])
{
    QCoreApplication::setApplicationVersion(QStringLiteral(PPRZ_VERSION_DESC));
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_play"));
    QCoreApplication::setApplicationName(QStringLiteral("Paparazzi replay"));
    qputenv("QT_LOGGING_RULES", "qt.qpa.wayland.textinput=false");

    QApplication app(argc, argv);
    QString iconPath = QStringLiteral(":/penguin_icon_rep.png");
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(QApplication::desktopFileName(),
                                   QStringLiteral("Paparazzi replay"),
                                   QStringLiteral("Player to replay flights"),
                                   iconPath,
                                   QStringLiteral("paparazzi-play"));
    QApplication::setWindowIcon(icon);

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Paparazzi Replay"));
    parser.addHelpOption();
    parser.addOption(
            QCommandLineOption(QStringList() << QStringLiteral("v") << QStringLiteral("version"),
                               QStringLiteral("Displays version information.")));
    parser.addOption(QCommandLineOption(QStringList() << QStringLiteral("b"),
                                        QStringLiteral("Ivy Bus. Default is 127.255.255.255:2010"),
                                        QStringLiteral("bus"),
                                        QStringLiteral("127.255.255.255:2010")));
    parser.addOption(QCommandLineOption(QStringList() << QStringLiteral("d"),
                                        QStringLiteral("Port Default is /dev/ttyUSB0"),
                                        QStringLiteral("port"),
                                        QStringLiteral("/dev/ttyUSB0")));
    parser.addOption(QCommandLineOption(QStringList() << QStringLiteral("o"),
                                        QStringLiteral("Output binary messages on serial port")));
    parser.addOption(QCommandLineOption(QStringList() << QStringLiteral("s"),
                                        QStringLiteral("Baudrate Default is 9600"),
                                        QStringLiteral("baudrate"),
                                        QStringLiteral("9600")));
    parser.addOption(
            QCommandLineOption(QStringList() << QStringLiteral("shfc"),
                               QStringLiteral("Enable UART hardware flow control (CTS/RTS)")));
    parser.addOption(
            QCommandLineOption(QStringList() << QStringLiteral("no-gui"),
                               QStringLiteral("Run without GUI (equivalent to play-nox)")));
    parser.addOption(QCommandLineOption(QStringList() << QStringLiteral("consumer-messages"),
                                        "Validate replay payloads against this messages.xml -- the "
                                        "dictionary your GCS/server uses to "
                                        "decode them. Frames whose integer fields do not match it "
                                        "are skipped, preventing consumer "
                                        "int_of_string errors on version-mismatched logs. Default: "
                                        "$PAPARAZZI_HOME/var/messages.xml.",
                                        QStringLiteral("file")));

    parser.addPositionalArgument(QStringLiteral("log"),
                                 QStringLiteral("Log file to load."),
                                 QStringLiteral("[log file]"));

    // Robust Command-Line Argument Parsing (Ported from LogPlotter)
    // Resolves issues where external launchers incorrectly fragment quoted file paths or arguments.
    QStringList argsList = QApplication::arguments();
    QStringList mergedArgs;
    for (int i = 0; i < argsList.size(); ++i) {
        QString arg = argsList[i];
        if ((arg.startsWith('\'') && !arg.endsWith('\''))
            || (arg.startsWith('"') && !arg.endsWith('"'))) {
            QChar quoteType = arg[0];
            QString merged = arg;
            int j = i + 1;
            bool foundClosed = false;
            while (j < argsList.size()) {
                merged += QStringLiteral(" ") + argsList[j];
                if (argsList[j].endsWith(quoteType)) {
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

    if (parser.isSet(QStringLiteral("version"))) {
        parser.showVersion();
        return 0;
    }

    if (parser.isSet(QStringLiteral("o"))) {
        qWarning() << "Notice: Replaying to physical binary serial out (-o) is not fully "
                      "implemented in this Qt architecture revision.";
    }

    QString ivyBus = parser.value(QStringLiteral("b"));
    if (ivyBus.isEmpty())
        ivyBus = QStringLiteral("127.255.255.255:2010");
    bool noGui = parser.isSet(QStringLiteral("no-gui"));
    QStringList args = parser.positionalArguments();

    PlayCore core;
    core.setIvyBus(ivyBus);
    core.setNoGui(noGui);
    core.setConsumerMessages(parser.value(QStringLiteral("consumer-messages")));
    core.startIvy();

    PlayWindow *window = nullptr;
    if (!noGui) {
        window = new PlayWindow(&core);
        window->show();
    }

    if (!args.isEmpty()) {
        if (core.loadLog(args.first())) {
            double s;
            double e;
            core.getBounds(s, e);
            if (window) {
                window->setWindowTitle(QStringLiteral("Replay: ")
                                       + QFileInfo(args.first()).fileName());
            }
            core.play();
        }
    }

    return QApplication::exec();
}

#include "play.moc"
