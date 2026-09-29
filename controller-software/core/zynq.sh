#!/usr/bin/env bash
# Deploy controller_core to the Zynq and run it, over ssh.
#
# usage: ./zynq.sh deploy            stop the controller, copy bin/, config/ and monitor/ from
#                                    zynq_files to the board (config/temp/ is left alone)
#        ./zynq.sh start [config]    start controller_core in the background, from the board's home
#                                    directory, logging to /var/volatile/controller.log (RAM)
#                                    (default config: controller/config/user/config.json),
#                                    and the monitor web page on port 8080 (ZYNQ_MONITOR_PORT)
#        ./zynq.sh stop              stop both
#        ./zynq.sh status            is it running, and the last lines of the log
#        ./zynq.sh log               follow the log (Ctrl-C stops following, not the controller)
#
# The board is $ZYNQ_HOST (default "zynq", an ssh alias, see build_and_zynq_setup.md).
# Starting and stopping need sudo on the board: it asks for the password on this terminal,
# or set ZYNQ_SUDO_PASSWORD to answer it non-interactively.
set -euo pipefail

CORE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST="${ZYNQ_HOST:-zynq}"
# the log is in RAM (tmpfs): the realtime loop writes its prints straight to it, and a write to a file on
# the SD card blocks for ~7 ms whenever the kernel flushes the card (every 5 s), which misses FPGA updates.
# It is lost on reboot.
LOG=/var/volatile/controller.log
MONITOR_PORT="${ZYNQ_MONITOR_PORT:-8080}"

# single-quote for a remote shell (sed rather than ${//}, which bash 3.2 on macOS gets wrong)
quote() { printf "'%s'" "$(printf '%s' "$1" | sed "s/'/'\\\\''/g")"; }

# run a command as root on the board (busybox sh)
as_root() {
    if [[ -n "${ZYNQ_SUDO_PASSWORD:-}" ]]; then
        printf '%s\n' "$ZYNQ_SUDO_PASSWORD" | ssh "$HOST" "sudo -S -p '' sh -c $(quote "$1")"
    else
        ssh -t "$HOST" "sudo sh -c $(quote "$1")"
    fi
}

remote_home() { ssh "$HOST" 'echo $HOME'; }

# the pattern is written so it does not match the shell running pkill
stop() { as_root "pkill -f '[c]ontroller/bin/controller_core' || true"; }

# procps ps only lists the current session without -e
running() { ssh "$HOST" "ps -ef | grep -q '[c]ontroller/bin/controller_core'"; }

# the monitor web server (controller/monitor/em_monitor.py) runs as the login user: it only reads
monitor_running() { ssh "$HOST" "ps -ef | grep -q '[c]ontroller/monitor/em_monitor.py'"; }
monitor_stop() { ssh "$HOST" "pkill -f '[c]ontroller/monitor/em_monitor.py' || true"; }
monitor_url() { echo "http://$(ssh -G "$HOST" < /dev/null 2> /dev/null | awk '$1 == "hostname" {print $2}'):$MONITOR_PORT/"; }
monitor_start() {
    monitor_running && return 0
    # low priority on CPU 0: controller_core's realtime thread has CPU 1
    ssh "$HOST" "cd ~ && trap '' HUP && (setsid nohup nice -n 10 taskset 1 python3 controller/monitor/em_monitor.py --port $MONITOR_PORT > /var/volatile/monitor.log 2>&1 < /dev/null &)"
    sleep 1
    if monitor_running; then
        echo "==> monitor: $(monitor_url)"
    else
        echo "error: the monitor did not start:" >&2
        ssh "$HOST" "tail -n 20 /var/volatile/monitor.log" >&2
    fi
}

case "${1:-}" in
    deploy)
        running && { echo "==> stopping controller_core"; stop; sleep 1; }
        monitor_stop
        echo "==> copying bin/ and config/ to $HOST"
        # keep the binary that was on the board before the first deploy, in case it needs restoring
        ssh "$HOST" '[ -e controller/bin/controller_core.orig ] || [ ! -e controller/bin/controller_core ] || cp -p controller/bin/controller_core controller/bin/controller_core.orig'
        COPYFILE_DISABLE=1 tar -C "$CORE/zynq_files" --no-xattrs --no-mac-metadata \
            --exclude controller/config/temp --exclude 'controller/config/fpga_configs/*/build' --exclude .DS_Store --exclude __pycache__ \
            -cf - controller/bin controller/config controller/monitor | ssh "$HOST" 'tar -xf - -C ~'
        ssh "$HOST" 'ls -la controller/bin; ls controller/config controller/config/fpga_configs controller/monitor'
        ;;
    start)
        config="${2:-controller/config/user/config.json}"
        home="$(remote_home)"
        ssh "$HOST" "[ -f $(quote "$config") ]" || { echo "error: no $config on $HOST" >&2; exit 1; }
        running && { echo "error: controller_core is already running, use ./zynq.sh stop" >&2; exit 1; }
        # the board has no RTC/NTP, so set its clock from this machine (log timestamps, web certs).
        # SIGHUP is ignored before backgrounding: with the password prompt (ssh -t) the terminal hangs up
        # as soon as this returns, which otherwise kills controller_core before setsid/nohup take effect
        as_root "date -u -s @$(date -u +%s) >/dev/null && cd $(quote "$home") && trap '' HUP && (setsid nohup controller/bin/controller_core $(quote "$config") > $LOG 2>&1 < /dev/null &)"
        sleep 3
        if running; then echo "==> controller_core running with $config"; else echo "error: controller_core exited, see below" >&2; fi
        ssh "$HOST" "tail -n 25 $LOG"
        monitor_start
        running
        ;;
    stop)
        stop
        monitor_stop
        running && { echo "error: still running" >&2; exit 1; } || echo "==> stopped"
        ;;
    status)
        if running; then echo "==> running: $(ssh "$HOST" "ps -ef | grep '[c]ontroller/bin/controller_core'")"; else echo "==> not running"; fi
        if monitor_running; then echo "==> monitor: $(monitor_url)"; else echo "==> monitor not running"; fi
        ssh "$HOST" "[ -f $LOG ] && tail -n 10 $LOG || true"
        ;;
    log)
        ssh -t "$HOST" "tail -n 50 -f $LOG"
        ;;
    *)
        sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
        exit 1
        ;;
esac
