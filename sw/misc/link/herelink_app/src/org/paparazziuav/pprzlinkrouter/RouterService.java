package org.paparazziuav.pprzlinkrouter;

import android.app.Notification;
import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Collections;

/**
 * Runs pprzlink_router between the Herelink radio and the laptop, and restarts it when it stops
 * or when the laptop connection changes (USB tethering 192.168.42.x or Wi-Fi hotspot 192.168.43.x).
 */
public class RouterService extends Service {
  private static final String TAG = "pprzlink_router";
  private static final String RADIO = "udp://192.168.0.11:5242:192.168.0.10:5243";
  private static final String USB_BCAST = "192.168.42.255";
  private static final String WIFI_BCAST = "192.168.43.255";

  private volatile boolean running = false;
  private Process router = null;
  private String routerBcast = null;

  @Override
  public int onStartCommand(Intent intent, int flags, int startId) {
    Notification n = new Notification.Builder(this)
        .setContentTitle("pprzlink router")
        .setContentText("Forwarding the Paparazzi telemetry")
        .setSmallIcon(android.R.drawable.stat_sys_upload)
        .build();
    startForeground(1, n);

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
  }

  @Override
  public IBinder onBind(Intent intent) {
    return null;
  }

  /** Check every 2 s that the router runs with the broadcast address of the active laptop link */
  private void watch() {
    while (running) {
      String bcast = laptopBroadcast();
      if (router == null || !isAlive(router) || !bcast.equals(routerBcast)) {
        stopRouter();
        startRouter(bcast);
      }
      try {
        Thread.sleep(2000);
      } catch (InterruptedException e) {
        return;
      }
    }
  }

  /** USB tethering if active, else the Wi-Fi hotspot */
  private String laptopBroadcast() {
    try {
      for (NetworkInterface ni : Collections.list(NetworkInterface.getNetworkInterfaces())) {
        for (InetAddress a : Collections.list(ni.getInetAddresses())) {
          if (a instanceof Inet4Address && a.getHostAddress().startsWith("192.168.42.")) {
            return USB_BCAST;
          }
        }
      }
    } catch (Exception e) {
      Log.w(TAG, "Could not list the network interfaces", e);
    }
    return WIFI_BCAST;
  }

  private void startRouter(String bcast) {
    try {
      File bin = extractRouter();
      router = new ProcessBuilder(bin.getAbsolutePath(), "-e", RADIO,
                                  "-e", "udp://0.0.0.0:4243:" + bcast + ":4242")
          .redirectErrorStream(true).start();
      routerBcast = bcast;
      Log.i(TAG, "Started the router to " + bcast);
    } catch (Exception e) {
      Log.e(TAG, "Could not start the router", e);
      router = null;
    }
  }

  private void stopRouter() {
    if (router != null) {
      router.destroy();
      router = null;
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
