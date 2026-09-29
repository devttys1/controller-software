### Build Setup

install cross-platform compilers:
`sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf`

install build tools: 
`sudo apt update`
`sudo apt install build-essential ninja-build`

install cmake
`sudo apt install cmake`

install debugger:
`sudo apt install gdb-multiarch`

### ZYNQ SSH Setup

ZYNQ username: `em-os`
password: `0`

create ssh keys:
`ssh-keygen -t rsa -b 4096 -C "your_email@example.com"`
save them in the `core/.ssh` folder

copy ssh key to zynq:
`ssh-copy-id -i .ssh/zynq em-os@192.168.1.123` (your zynq's IP address)

remove past configs if zynq has been re-imaged
`ssh-keygen -f "/home/[user]/.ssh/known_hosts" -R "192.168.1.123"` (your zynq's IP address)

create ssh config:
`mkdir -p ~/.ssh && chmod 700 ~/.ssh`

create config:
`touch ~/.ssh/config`

set permissions:
`chmod 600 ~/.ssh/config`

configure ssh settings:
`nano ~/.ssh/config`

add config:
(replace with actual IP and path to the keys)
```
Host zynq
    HostName 192.168.1.123
    User em-os
    IdentityFile /home/[user]/controller-software/controller-software/core/.ssh/zynq 
    Port 22
```

optional, reach the zynq by name instead of IP (if your router registers DHCP hostnames in DNS):
on the zynq, add `hostname em-os` under `iface eth0 inet dhcp` in `/etc/network/interfaces` and reboot,
then use `HostName em-os.<your local domain>` in the ssh config above

build project to copy files to zynq

TODO: figure out a way to better handle the permissions, currently you must enter the password each time (maybe not that bad?)

### ZYNQ Setup (run through ssh):

allow write access so we can update it later from vs code (note this is unsafe since a malicious bin could be added)
`sudo chmod u+w controller/bin/controller_core`


update zynq node modules
in controller/web, run `npm install`

update zynq time (required for certs to be valid)
`sudo date -s "2025-3-21 18:37:00"` (use the current time)


### Deploy and run from the command line (`zynq.sh`)

`zynq.sh` in this directory does the same as the VS Code tasks from any terminal (macOS or Linux), using the `zynq` ssh alias above (override with `ZYNQ_HOST`):

```sh
./docker_build.sh                  # or the VS Code / CMake build: puts the binary in zynq_files/controller/bin
./zynq.sh deploy                   # stop controller_core, copy zynq_files/controller/{bin,config} to the board
                                   # (config/temp is left alone; the first deploy keeps the board's binary as controller_core.orig)
./zynq.sh start controller/config/spindle_bench/config.json    # default: controller/config/user/config.json
./zynq.sh status                   # running or not, last lines of the log
./zynq.sh log                      # follow the log (Ctrl-C stops following, not the controller)
./zynq.sh stop
```

`start` also starts the monitor web page and prints its address, `http://<board>:8080/` (see Monitoring below).

- `start` runs `controller_core` as root in the background from the board's home directory, logging to `/var/volatile/controller.log`, which is in RAM and lost on reboot. Don't redirect it to the SD card: the realtime loop writes its prints directly, and a write that lands while the kernel flushes the card (every 5 s) blocks for about 7 ms, so the controller misses FPGA updates ("FPGA interrupt missed"). The optional argument is the main config file; relative paths are from `/home/em-os`.
- `start` also sets the board's clock from your machine, since the board has no RTC or NTP.
- `start`/`stop` need sudo on the board: you are asked for the em-os password, or set `ZYNQ_SUDO_PASSWORD` to answer it non-interactively.

### Monitoring

**Monitor web page:** `http://em-os.<your local domain>:8080/` (your board's name), started by `./zynq.sh start`.
- **What it shows:** each Yaskawa encoder live at the full 1 kHz: speed, position, electrical angle with the hall sectors, hall state, hall reference, and frame errors and no responses. Charts cover the last 1, 5 or 15 s, and the controller log is at the bottom.
- **Download CSV:** saves the last 16 s of samples, every cycle, including the raw `single_turn` and `timestamp_fast`.
- **How it works:**
  - The controller's driver writes every cycle into a ring buffer in shared memory, `/dev/shm/em_<driver>_<n>` (`controller/inc/telemetry_ring.h`). That is a plain memory write, so it cannot stall the realtime loop.
  - `zynq_files/controller/monitor/em_monitor.py` serves the page and streams the ring to it. It uses Python's standard library, which is on the board.
  - It runs as `em-os` at low priority on CPU 0, and uses about 5% of that CPU per open page.
- **Access:** it is read-only and has no login, so keep it on the bench network. `ZYNQ_MONITOR_PORT` changes the port.
- **Trying it without a board:** `python3 tests/fake_monitor_rings.py /tmp/em_shm &` writes a simulated spindle. Then run `python3 zynq_files/controller/monitor/em_monitor.py --shm-dir /tmp/em_shm` and open `http://localhost:8080/`.

**Node.js web interface:** the controller also starts the web server in `controller/web` with Node.js. The current em-os image does not include Node.js, so there is no web interface on port 3000 yet. The "Nodejs webserver started" log line only means it tried.

**Log:** `./zynq.sh log` follows it. For the Yaskawa encoders, set `debug_print_interval_ms` in the `node_4_yaskawa_encoders` section of the driver config to print each encoder periodically. The `spindle_bench` config does this every 500 ms.

To check the FPGA is cycling, the update interrupts should increase by about the update frequency per second:
`grep fpga_mem_update /proc/interrupts` (run twice, a second apart).

### Petalinux Setup:
package images:
`petalinux-package --boot --fsbl --u-boot --force`

create disk image:
`petalinux-package --wic --outdir /home/[your_output_dir] --wic-extra-args "-c xz" -b "BOOT.BIN,image.ub,boot.scr" --wks project-spec/configs/rootfs.wks`





