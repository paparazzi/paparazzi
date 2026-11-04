package org.paparazziuav.pprzlinkrouter;

import android.app.Notification;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
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

/**
 * Runs pprzlink_router between the Herelink radio and the laptops, and restarts it when it stops.
 *
 * The router finds the laptops itself (auto endpoint): it sends the telemetry to every laptop running
 * Paparazzi's link that it heard from, on any network the handheld is on (USB tethering, its Wi-Fi hotspot,
 * or a Wi-Fi network it joined), and a 1 Hz beacon to the broadcast addresses so new laptops find it.
 */
public class RouterService extends Service {
  private static final String TAG = "pprzlink_router";
  private static final String RADIO = "udp://192.168.0.11:5242:192.168.0.10:5243";
  private static final String GCS = "udp://0.0.0.0:4243:auto:4242";

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

  /** Every second: restart the router if it stopped, and give it the Wi-Fi RSSI */
  private void watch() {
    while (running) {
      if (router == null || !isAlive(router)) {
        stopRouter();
        startRouter();
      }
      sendWifiQuality();
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
        routerIn.write(("rssi " + info.getRssi() + "\n").getBytes());
        routerIn.flush();
      }
    } catch (Exception e) {
      Log.w(TAG, "Could not send the Wi-Fi RSSI to the router", e);
    }
  }

  private void startRouter() {
    try {
      File bin = extractRouter();
      router = new ProcessBuilder(bin.getAbsolutePath(), "-p", "-e", RADIO, "-e", GCS)
          .redirectErrorStream(true).start();
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

  private void stopRouter() {
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
