#!/usr/bin/env python3
# mt333x_config.py - Change settings on a MediaTek MTK3339 GPS receiver.
#
# Companion to mt333x_probe.py (read-only). Every command here MUTATES
# receiver state: stop gpsd first so the port is free, and know that most
# settings are volatile (lost on power loss unless VBACKUP battery fitted).
#
# Refused by design (not offered as subcommands):
#   PMTK104 full-cold without --confirm (wipes all config to factory),
#   PMTK184 LOCUS flash erase, PMTK225 periodic/backup modes (need
#   supporting hardware; can strand a headless host), baud > 115200
#   (NMEA port max per Rev.A01; higher rates are Download-Agent only),
#   boot-ROM handshake / firmware flash (use mt333x_fw_update.py).
#
# Usage: mt333x_config.py SERIAL_PORT SUBCOMMAND [options]
#   --dryrun prints the packet without opening the port.
#   --verbose shows TX/RX lines. Every mutating command waits for the
#   $PMTK001,<cmd>,<flag> ACK and fails closed (nonzero exit) unless flag=3.
"""Change settings on a MediaTek MTK3339 GPS receiver (mutating)."""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None  # --dryrun / --help work without pyserial

from mt333x_probe import pmtk_packet, query

NMEA_BAUDS = (4800, 9600, 14400, 19200, 38400, 57600, 115200)
NAV_THRESHOLDS = ("0", "0.2", "0.4", "0.6", "0.8", "1.0", "1.5", "2.0")
# PMTK314 field order: GLL RMC VTG GGA GSA GSV then reserved, ZDA(17) MCHN(18)
SENTENCES = ("gll", "rmc", "vtg", "gga", "gsa", "gsv", "zda", "mchn")
RESTARTS = {"hot": "PMTK101", "warm": "PMTK102", "cold": "PMTK103",
            "full-cold": "PMTK104"}


def ack_flag(replies, cmd):
    """Extract the $PMTK001,<cmd>,<flag> flag from replies; None if absent."""
    for r in replies:
        parts = r.split("*")[0].split(",")
        if len(parts) == 3 and parts[0] == "$PMTK001" and parts[1] == cmd:
            try:
                return int(parts[2])
            except ValueError:
                pass
    return None


def send_checked(ser, body, verbose):
    """Send a set-command, wait for its ACK, fail closed unless flag=3."""
    cmd = body.split(",")[0].replace("PMTK", "")
    replies = query(ser, body, 3.0, verbose)
    flag = ack_flag(replies, cmd)
    meanings = {"0": "invalid packet", "1": "unsupported on this firmware",
                "2": "valid but action failed", "3": "ok"}
    if flag is None:
        print("no ACK for PMTK%s (replies: %s)" % (cmd, replies or "none"))
        return 1
    print("PMTK%s: %s (flag=%d)" % (cmd, meanings.get(str(flag), "?"), flag))
    return 0 if flag == 3 else 1


def build_nmea_out(args):
    """PMTK314 body from per-sentence intervals (0=off, 1-5=every-nth-fix)."""
    vals = {"gll": 0, "rmc": 1, "vtg": 0, "gga": 1, "gsa": 1, "gsv": 5,
            "zda": 1, "mchn": 0}
    for s in SENTENCES:
        v = getattr(args, s)
        if v is not None:
            if not 0 <= v <= 5:
                raise ValueError("%s interval must be 0-5" % s)
            vals[s] = v
    fields = [str(vals["gll"]), str(vals["rmc"]), str(vals["vtg"]),
              str(vals["gga"]), str(vals["gsa"]), str(vals["gsv"])]
    fields += ["0"] * 11 + [str(vals["zda"]), str(vals["mchn"])]
    return "PMTK314," + ",".join(fields)


def build_rate(args):
    """PMTK220 body from --hz (EASY aiding only works at 1 Hz)."""
    if not 1 <= args.hz <= 10:
        raise ValueError("rate must be 1-10 Hz")
    ms = int(1000 / args.hz)
    if not 100 <= ms <= 10000:
        raise ValueError("interval %d ms out of range 100-10000" % ms)
    if args.hz != 1:
        print("WARNING: EASY self-ephemeris only works at 1 Hz", file=sys.stderr)
    return "PMTK220,%d" % ms


def add_common(sub, needs_confirm=False, confirm_text=""):
    """Attach --dryrun/--verbose/--confirm flags shared by subcommands."""
    sub.add_argument("-v", "--verbose", action="store_true")
    sub.add_argument("--dryrun", action="store_true",
                     help="print packet without opening the port")
    if needs_confirm:
        sub.add_argument("--confirm", action="store_true",
                         help=confirm_text or "required to apply")


def require_confirm(args, what):
    """Abort unless --confirm was passed."""
    if not args.confirm:
        print("refusing %s without --confirm" % what)
        return False
    return True


def open_port(port, baud):
    """Open the serial port; None with an error message on failure."""
    if serial is None:
        print("ERROR: pyserial is not installed", file=sys.stderr)
        return None
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as exc:  # port busy (gpsd?), missing, permission
        print("ERROR: cannot open %s: %s" % (port, exc), file=sys.stderr)
        print("hint: sudo systemctl stop gpsd gpsd.socket first",
              file=sys.stderr)
        return None
    time.sleep(0.3)
    return ser


def run_body(args, body):
    """Dry-run print or open port, send body, report ACK status."""
    if args.dryrun:
        print(pmtk_packet(body).decode().strip())
        return 0
    ser = open_port(args.serial_port, args.baud)
    if ser is None:
        return 2
    try:
        return send_checked(ser, body, args.verbose)
    finally:
        ser.close()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Change settings on a MediaTek MTK3339 GPS receiver. "
                    "Stop gpsd first; settings are volatile unless VBACKUP "
                    "battery is fitted.",
        epilog="No subcommand: show current config (read-only).")
    parser.add_argument("serial_port", help="serial device, e.g. /dev/ttyAMA0")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="port baud rate (default: 115200)")
    sub = parser.add_subparsers(dest="cmd")

    p = sub.add_parser("show", help="show current config (read-only)")
    p.add_argument("-v", "--verbose", action="store_true")
    p.add_argument("--dryrun", action="store_true",
                     help="print query packets without opening the port")

    p = sub.add_parser("nmea-out", help="set NMEA sentence intervals (PMTK314)")
    for s in SENTENCES:
        p.add_argument("--" + s, type=int, metavar="0-5", default=None)
    p.add_argument("--defaults", action="store_true",
                   help="restore default sentence set")
    add_common(p)

    p = sub.add_parser("rate", help="set fix update rate (PMTK220)")
    p.add_argument("--hz", type=float, required=True, help="1-10 Hz")
    add_common(p)

    p = sub.add_parser("baud", help="set NMEA baud rate (PMTK251)")
    p.add_argument("--baud-rate", type=int, required=True,
                   choices=NMEA_BAUDS)
    add_common(p, True, "link drops; reconnect at the new rate after")

    p = sub.add_parser("sbas", help="SBAS enable + mode (PMTK313/319)")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--enable", action="store_true")
    g.add_argument("--disable", action="store_true")
    p.add_argument("--mode", choices=("test", "integrity"))
    add_common(p)

    p = sub.add_parser("dgps", help="DGPS source mode (PMTK301)")
    p.add_argument("--mode", required=True, choices=("none", "rtcm", "waas"))
    add_common(p)

    p = sub.add_parser("datum", help="geodetic datum (PMTK330)")
    p.add_argument("--datum", type=int, required=True,
                   help="0=WGS84 (1=Tokyo-M, 2=Tokyo-A, ... up to 221)")
    add_common(p, True, "non-WGS84 datum offsets all positions")

    p = sub.add_parser("nav-threshold",
                       help="static-drift freeze threshold (PMTK386)")
    p.add_argument("--ms", required=True, choices=NAV_THRESHOLDS,
                   help="m/s, 0 disables")
    add_common(p)

    p = sub.add_parser("qzss", help="QZSS NMEA format + function (PMTK351/352)")
    p.add_argument("--nmea", choices=("enable", "disable"))
    p.add_argument("--func", choices=("enable", "disable"))
    add_common(p)

    p = sub.add_parser("aic", help="jammer rejection (PMTK286, default on)")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--enable", action="store_true")
    g.add_argument("--disable", action="store_true")
    add_common(p)

    p = sub.add_parser("easy", help="self-ephemeris aiding (PMTK869)")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--enable", action="store_true")
    g.add_argument("--disable", action="store_true")
    add_common(p)

    p = sub.add_parser("locus", help="logger start/stop/interval (PMTK185/187)")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--start", action="store_true")
    g.add_argument("--stop", action="store_true")
    g.add_argument("--interval", type=int, metavar="SEC")
    add_common(p)

    p = sub.add_parser("restart", help="hot/warm/cold restart (PMTK101-104)")
    p.add_argument("--mode", required=True,
                   choices=sorted(RESTARTS))
    add_common(p, True, "full-cold wipes all config to factory defaults")

    p = sub.add_parser("standby", help="sleep until any byte sent (PMTK161)")
    add_common(p, True, "link goes silent; reopen port and send a byte to wake")
    args = parser.parse_args(argv)

    if args.cmd is None or args.cmd == "show":
        from mt333x_probe import QUERIES, pmtk_packet
        verbose = getattr(args, "verbose", False)
        if getattr(args, "dryrun", False):
            for _, body, _ in QUERIES:
                print(pmtk_packet(body).decode().strip())
            return 0
        ser = open_port(args.serial_port, args.baud)
        if ser is None:
            return 2
        try:
            for name, body, _ in QUERIES:
                for r in query(ser, body, 2.5, verbose):
                    print("  %-8s %s" % (name, r[:200]))
            return 0
        finally:
            ser.close()

    try:
        if args.cmd == "nmea-out":
            if args.defaults and any(getattr(args, s) is not None for s in SENTENCES):
                print("ERROR: --defaults conflicts with per-sentence intervals", file=sys.stderr)
                return 2
            body = ("PMTK314,-1" if args.defaults else build_nmea_out(args))
        elif args.cmd == "rate":
            body = build_rate(args)
        elif args.cmd == "baud":
            if not require_confirm(args, "baud change"):
                return 2
            body = "PMTK251,%d" % args.baud_rate
            print("after ACK, reconnect at %d baud" % args.baud_rate, file=sys.stderr)
        elif args.cmd == "sbas":
            bodies = ["PMTK313,%d" % (1 if args.enable else 0)]
            if args.mode:
                bodies.append("PMTK319,%d" % (0 if args.mode == "test" else 1))
            return run_multi(args, bodies)
        elif args.cmd == "dgps":
            body = "PMTK301,%d" % ({"none": 0, "rtcm": 1, "waas": 2}[args.mode])
        elif args.cmd == "datum":
            if not 0 <= args.datum <= 221:
                raise ValueError("datum must be 0-221")
            if args.datum != 0 and not require_confirm(args, "non-WGS84 datum"):
                return 2
            body = "PMTK330,%d" % args.datum
        elif args.cmd == "nav-threshold":
            body = "PMTK386,%s" % args.ms
        elif args.cmd == "qzss":
            if args.nmea is None and args.func is None:
                print("nothing to do: pass --nmea and/or --func")
                return 2
            bodies = []
            if args.nmea:
                bodies.append("PMTK351,%d" % (1 if args.nmea == "enable" else 0))
            if args.func:
                # PMTK352 polarity is inverted: 0=enable, 1=disable
                bodies.append("PMTK352,%d" % (0 if args.func == "enable" else 1))
            return run_multi(args, bodies)
        elif args.cmd == "aic":
            body = "PMTK286,%d" % (1 if args.enable else 0)
        elif args.cmd == "easy":
            body = "PMTK869,1,%d" % (1 if args.enable else 0)
        elif args.cmd == "locus":
            if args.interval is not None:
                body = "PMTK187,1,%d" % args.interval
            else:
                body = "PMTK185,%d" % (0 if args.start else 1)
        elif args.cmd == "restart":
            if args.mode == "full-cold" and not require_confirm(
                    args, "full-cold (wipes config)"):
                return 2
            body = RESTARTS[args.mode]
        elif args.cmd == "standby":
            if not require_confirm(args, "standby"):
                return 2
            body = "PMTK161,0"
            print("wake: reopen the port and send any byte", file=sys.stderr)
        else:
            parser.error("unknown command")
    except ValueError as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        return 2
    return run_body(args, body)


def run_multi(args, bodies):
    """Send several set-commands in order; stop at first failure."""
    if args.dryrun:
        for b in bodies:
            print(pmtk_packet(b).decode().strip())
        return 0
    ser = open_port(args.serial_port, args.baud)
    if ser is None:
        return 2
    try:
        rc = 0
        for b in bodies:
            rc = send_checked(ser, b, args.verbose)
            if rc != 0:
                break
        return rc
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
