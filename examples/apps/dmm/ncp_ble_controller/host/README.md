# DMM NCP+BLE Controller — Host Setup Guides

Host-side setup and test instructions for the TI cc27xx DMM NCP+BLE Controller firmware.

---

## [Ubuntu PC Host Setup Guide](README_UBUNTU_HOST.html)

Step-by-step instructions for Ubuntu 22.04 (Python 3.11). Covers the Combined Serial
MUX bridge, BlueZ HCI adapter attachment, OTBR agent startup, Thread network
operations via `ot-ctl`, and BLE HCI validation.

## [BeagleBone Black Host Setup Guide](README_BBB_HOST.md)

Step-by-step instructions for BeagleBone Black running Debian 10 Buster (Python 3.7).
Covers offline package installation, the MUX bridge, and BLE HCI validation —
no internet connection required on the BBB.
