# Commissioning a rig board

A roster entry is commissioned when its serial, its UF2 drive and its re-entry path are proven on the board; until
then it has no `nrf52.reentry` and `run` refuses it.

1. Host: `JLinkExe` and `mcopy` (mtools) on `PATH`; no automounter takes the drive (`udisks2` absent, or a udev rule
   setting `UDISKS_IGNORE=1` for the board's VID:PID); the nRF USB port (DK: J3) cabled to the host.
2. Hold the lock (SKILL.md Reservation), then read the serial with one J-Link session, `mem32 10000060 2`
   (FICR DEVICEID[0], DEVICEID[1]: nRF52840 PS v1.7 4.4.1.3). The bootloader's serial is DEVICEID[1] then
   DEVICEID[0] as 16 upper-case hex digits; it is the entry's `uid`. Note UICR (`mem32 10001200 4`, `mem32 10001014
   2`) and the boot words other projects rely on before the first erase.
3. Add the entry without `nrf52.reentry`, commit, build the record (SKILL.md Build) and run `install --under-hold`:
   it must pass, with the drive found by that serial and `INFO_UF2.TXT` matching.
4. Set `"nrf52": {"reentry": "gpregret"}`, commit, rebuild the record, release the hold and run `run` twice, then
   `run --fail-at copy` (cleanup passes, exit 1, board released).
5. Hand back: run each other project's HIL on the board once (tinyusb's, the Arduino HILs' when their runners are idle)
   and record their result with the commit that commissioned the entry.
