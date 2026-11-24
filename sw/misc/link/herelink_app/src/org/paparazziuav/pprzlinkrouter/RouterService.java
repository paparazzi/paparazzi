package org.paparazziuav.pprzlinkrouter;

import android.app.Notification;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.BatteryManager;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.IBinder;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Runs pprzlink_router between the Herelink radio and the laptops, and restarts it when it stops.
 *
 * The router finds the laptops itself (auto endpoint): it sends the telemetry to every laptop running
 * Paparazzi's link that it heard from, on any network the handheld is on (USB tethering, its Wi-Fi hotspot,
 * or a Wi-Fi network it joined), and a 1 Hz beacon to the broadcast addresses so new laptops find it.
 *
 * It gives the router the values for its LINK_STATS on stdin: the Wi-Fi RSSI (rssi), the radio RSRP of the
 * handheld ("Controller M" in the Herelink app, rssi_ac, read from the radio log: needs READ_LOGS, granted
 * with adb shell pm grant org.paparazziuav.pprzlinkrouter android.permission.READ_LOGS), the battery and the
 * temperature. The router logs all frames in the app's folder on the storage (LOG_SIZE_MB at most).
 */
public class RouterService extends Service {
  private static final String TAG = "pprzlink_router";
  private static final String RADIO = "udp://192.168.0.11:5242:192.168.0.10:5243";
  private static final String GCS = "udp://0.0.0.0:4243:auto:4242";
  private static final String LOG_SIZE_MB = "1024";
  private static final Pattern RSRP = Pattern.compile("RSRP_GCS_M (-?\\d+)");

  private volatile boolean running = false;
  private Process router = null;
  private OutputStream routerIn = null;
  private WifiManager.WifiLock wifiLock = null;

  @Override
  public int onStartCommand(Intent intent, int flags, int startId) {
    Notification n = new Notification.Builder(this)
        .setContentTitle("pprzlink router")
        .setContentText("Forwarding the Paparazzi telemetry")
        .setSmallIcon(android.R.drawable.stat_sys_upload)
        .build();
    startForeground(1, n);

    // Keep the Wi-Fi out of power save: as a Wi-Fi client it would otherwise add 100-300 ms of latency
    if (wifiLock == null) {
      WifiManager wm = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
      wifiLock = wm.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, TAG);
      wifiLock.setReferenceCounted(false);
      wifiLock.acquire();
    }

    if (!running) {
      running = true;
      new Thread(new Runnable() {
        public void run() { watch(); }
      }).start();
      new Thread(new Runnable() {
        public void run() { readRadio(); }
      }).start();
    }
    return START_STICKY;
  }

  @Override
  public void onDestroy() {
    running = false;
    stopRouter();
    if (wifiLock != null) {
      wifiLock.release();
      wifiLock = null;
    }
  }

  @Override
  public IBinder onBind(Intent intent) {
    return null;
  }

  /** Every second: restart the router if it stopped, and give it the Wi-Fi RSSI, battery and temperature */
  private void watch() {
    while (running) {
      if (router == null || !isAlive(router)) {
        stopRouter();
        startRouter();
      }
      sendWifiQuality();
      sendStatus();
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        return;
      }
    }
  }

  /** RSSI of the Wi-Fi network the handheld joined (not available as a hotspot) */
  private void sendWifiQuality() {
    if (routerIn == null) {
      return;
    }
    try {
      WifiManager wm = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
      WifiInfo info = wm.getConnectionInfo();
      if (info != null && info.getNetworkId() != -1) {
        send("rssi " + info.getRssi());
      }
    } catch (Exception e) {
      Log.w(TAG, "Could not read the Wi-Fi RSSI", e);
    }
  }

  /** Battery charge, and the highest temperature of the handheld (CPU, GPU, ...) */
  private void sendStatus() {
    Intent b = registerReceiver(null, new IntentFilter(Intent.ACTION_BATTERY_CHANGED));
    if (b != null) {
      int level = b.getIntExtra(BatteryManager.EXTRA_LEVEL, -1);
      int scale = b.getIntExtra(BatteryManager.EXTRA_SCALE, 100);
      if (level >= 0 && scale > 0) {
        send("battery " + (level * 100 / scale));
      }
    }
    int max = Integer.MIN_VALUE;
    File[] zones = new File("/sys/class/thermal").listFiles();
    if (zones != null) {
      for (File z : zones) {
        try {
          BufferedReader r = new BufferedReader(new java.io.FileReader(new File(z, "temp")));
          int t = Integer.parseInt(r.readLine().trim());
          r.close();
          max = Math.max(max, t > 1000 ? t / 1000 : t);  // millidegrees on most kernels
        } catch (Exception e) {
          // not a zone, or not readable
        }
      }
    }
    if (max == Integer.MIN_VALUE && b != null) {
      max = b.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0) / 10;  // battery temperature as fallback
    }
    if (max != Integer.MIN_VALUE) {
      send("temp " + max);
    }
  }

  /** The radio RSRP of the handheld, from the radio log (about once per second) */
  private void readRadio() {
    while (running) {
      Process p = null;
      try {
        p = new ProcessBuilder("logcat", "-b", "radio", "-v", "raw", "-T", "1", "-s", "RIL:D").start();
        BufferedReader r = new BufferedReader(new InputStreamReader(p.getInputStream()));
        String line;
        while (running && (line = r.readLine()) != null) {
          Matcher m = RSRP.matcher(line);
          if (m.find()) {
            send("rssi_ac " + m.group(1));
          }
        }
      } catch (Exception e) {
        Log.w(TAG, "Could not read the radio log", e);
      }
      if (p != null) {
        p.destroy();
      }
      try {
        Thread.sleep(5000);
      } catch (InterruptedException e) {
        return;
      }
    }
  }

  /** One line for the router's stdin */
  private synchronized void send(String line) {
    if (routerIn == null) {
      return;
    }
    try {
      routerIn.write((line + "\n").getBytes());
      routerIn.flush();
    } catch (Exception e) {
      Log.w(TAG, "Could not write to the router", e);
    }
  }

  private synchronized void startRouter() {
    try {
      File bin = extractRouter();
      File log = new File(getExternalFilesDir(null), "log");
      router = new ProcessBuilder(bin.getAbsolutePath(), "-p", "-e", RADIO, "-e", GCS,
          "-l", log.getAbsolutePath(), "-L", LOG_SIZE_MB).redirectErrorStream(true).start();
      routerIn = router.getOutputStream();
      logOutput(router.getInputStream());
      Log.i(TAG, "Started the router to auto");
    } catch (Exception e) {
      Log.e(TAG, "Could not start the router", e);
      router = null;
      routerIn = null;
    }
  }

  /** Copy the router output to logcat (also so it never blocks on a full pipe) */
  private static void logOutput(final InputStream out) {
    new Thread(new Runnable() {
      public void run() {
        try {
          BufferedReader r = new BufferedReader(new InputStreamReader(out));
          String line;
          while ((line = r.readLine()) != null) {
            Log.i(TAG, line.trim());
          }
        } catch (Exception e) {
          // the router stopped
        }
      }
    }).start();
  }

  private synchronized void stopRouter() {
    if (router != null) {
      router.destroy();
      router = null;
      routerIn = null;
    }
  }

  private static boolean isAlive(Process p) {
    try {
      p.exitValue();
      return false;
    } catch (IllegalThreadStateException e) {
      return true;
    }
  }

  /** Copy the router binary from the app to its files folder (once per app version) */
  private File extractRouter() throws Exception {
    File bin = new File(getFilesDir(), "pprzlink_router");
    long version = getPackageManager().getPackageInfo(getPackageName(), 0).lastUpdateTime;
    File stamp = new File(getFilesDir(), "router.version");
    if (!bin.exists() || !stamp.exists() || stamp.lastModified() != version) {
      InputStream in = getAssets().open("pprzlink_router");
      FileOutputStream out = new FileOutputStream(bin);
      byte[] buf = new byte[65536];
      int n;
      while ((n = in.read(buf)) > 0) {
        out.write(buf, 0, n);
      }
      out.close();
      in.close();
      new FileOutputStream(stamp).close();
      stamp.setLastModified(version);
    }
    bin.setExecutable(true, false);
    return bin;
  }
}
