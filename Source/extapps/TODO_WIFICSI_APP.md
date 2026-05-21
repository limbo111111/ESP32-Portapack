# Missing WiFi CSI Companion App Binary

The commit that introduced the "WiFi CSI motion detection" feature (commit `4cf5c1e`) added the backend ESP32 processing code in `ep_app_wificsi.cpp`, but forgot to include the compiled Portapack standalone app binary header (`wificsi.h`).

Because of this, the app did not appear on the HackRF Portapack menu.

## Temporary Placebo Fix
To solve the compilation and allow the app to show up on the Portapack UI menu, a placeholder binary (`wificsi.h`) was created based on the `espmanager.h` UI binary, and its name was changed to "WIFI CSI".

**This means the "WIFI CSI" app will appear on the Portapack menu and can receive backend data, but the GUI itself will look exactly like the ESP Manager app.**

## What needs to be done
A developer or AI with access to the Portapack firmware codebase (`portapack-mayhem`) needs to:
1. Create and compile the actual standalone app UI for "WiFi CSI Motion" for the Portapack ARM processor.
2. Convert that compiled binary into a C-style header array (like `unsigned char wificsi[] = { ... }`).
3. Replace the `Source/extapps/wificsi.h` placebo array with the real binary array.

Until this is done, the app is functional on the ESP backend side, but the Portapack GUI is a placeholder.
