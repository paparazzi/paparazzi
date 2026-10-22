#!/bin/bash
# Build the Herelink router app (no Gradle): needs default-jdk-headless aapt dalvik-exchange apksigner
# android-sdk-platform-23, and the cross-compiled router (make -C .. pprzlink_router_cc).
# usage: ./build.sh  ->  pprzlink_router.apk
set -e
cd "$(dirname "$0")"
SDK=/usr/lib/android-sdk/platforms/android-23/android.jar
rm -rf build && mkdir -p build/classes assets
cp ../pprzlink_router_cc assets/pprzlink_router
javac -source 8 -target 8 -bootclasspath $SDK -d build/classes $(find src -name '*.java') 2>&1 | grep -v "bootstrap class path" || true
dalvik-exchange --dex --output=build/classes.dex build/classes
aapt package -f -M AndroidManifest.xml -A assets -I $SDK -F build/unsigned.apk
(cd build && aapt add unsigned.apk classes.dex >/dev/null)
[ -f debug.keystore ] || keytool -genkeypair -keystore debug.keystore -storepass android -keypass android \
  -alias pprz -keyalg RSA -keysize 2048 -validity 10000 -dname "CN=Paparazzi" >/dev/null 2>&1
apksigner sign --ks debug.keystore --ks-pass pass:android --out pprzlink_router.apk build/unsigned.apk
echo "built $(pwd)/pprzlink_router.apk"
