# v2.0.1

### Bug Fixes

* Adding a delay after sending multipacket spi messages improves reliability! Some messages like sample rate got chopped up :rocket:

# v2.0.0

### New Features

* Added Wifi Shield Compatibility
* Added ability to change sample rates

### Breaking Changes

* Changed MCP3912 setup to NOT accept sample rate passed into it

## Release Candidate 4

### Bug Fixes

* Fix bug where accelerometer data was not placed in the right positions with WiFi.
* Went through every command and made sure a response was sent over wifi.
* Closes #209 - Impedance was not working on wifi, I created a custom packet type `0xC7` which is for impedance sent as `utf8` over wifi

## Release Candidate 3

### Bug Fixes

* LED light could have held in off state if connected to wifi shield while LED is off.

## Release Candidate 2

### Bug Fixes

* LED light could have held in off state if connected to wifi shield while LED is off.

### Enhancements

* Accelerometer enabled by default with wifi.

## Release Candidate 1

Initial release candidate

## Beta4

### Enhancements

* Needed to update sample rate setting functions to match the Cyton responses and thus work with wifi drivers.
* Add more helper printing functions to reduce code string foot prints such as `::printSampleRate()`, `::printFailure()`, and `::printSuccess()`

## Beta3

### New Features

* Send gains after connecting to Wifi shield

## Beta2

The overall goal was to clean the wifi code out of the library so it would not be needed when you are building a bare board.

### Bug Fixes

* Weird timing issue with wifi shield

### Breaking Changes

* Removed all wifi code and put into [new library](https://github.com/OpenBCI/OpenBCI_Wifi_Master_Library) that must be included! The new library is a called [OpenBCI_Wifi_Master_Library](https://github.com/OpenBCI/OpenBCI_Wifi_Master_Library). It is simply included when wifi is wanted.
* Removed `.loop()` function from library and all other `wifi*.()` functions.
* `DefaultGanglion.ino` now has wifi code.

### Files

* Removed `WifiGanglion.ino`
* Add `GanglionNoWifi.ino` example

## Beta1

* Initial release with wifi

# v1.1.2

### Bug fixes

* Changed the BLE on receive to use a ring buffer. See #2

# v1.1.1

* Changed the way SD file names are generated to allow for OTA programming.

# v1.0.0

* Initial Release
