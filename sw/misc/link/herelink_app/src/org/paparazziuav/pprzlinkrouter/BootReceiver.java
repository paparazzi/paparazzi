package org.paparazziuav.pprzlinkrouter;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

/** Start the router service when the Herelink ground unit has booted */
public class BootReceiver extends BroadcastReceiver {
  @Override
  public void onReceive(Context context, Intent intent) {
    context.startService(new Intent(context, RouterService.class));
  }
}
