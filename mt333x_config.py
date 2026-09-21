#!/usr/bin/env python3
# mt333x_config.py - Change settings on a MediaTek MTK3339 GPS receiver.
#
# Companion to mt333x_probe.py (read-only). Every command here MUTATES
# receiver state: stop gpsd first so the port is free, and know that most
# settings are volatile (lost on power loss unless VBACKUP battery fitted).
#
# Safety model per command: snapshot the current value via its documented
# query packet, print a concrete `revert:` command, send the set-command,
# wait for $PMTK001,<cmd>,3 (any other flag or no ACK fails closed), then
# re-read the query packet and diff (verify). `--no-verify` skips the
# re-read; `--dryrun` prints packets without opening the port.
#
# Refused by design (not offered as subcommands):
#   PMTK184 LOCUS flash erase, PMTK225 periodic/backup modes (need
#   supporting hardware; can strand a headless host), baud > 115200
#   (NMEA port max per Rev.A01; higher rates are Download-Agent only),
#   boot-ROM handshake / firmware flash (use mt333x_fw_update.py).
#
# Usage: mt333x_config.py SERIAL_PORT SUBCOMMAND [options]
"""Change settings on a MediaTek MTK3339 GPS receiver (mutating)."""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None  # --dryrun / --help work without pyserial

from mt333x_probe import QUERIES, pmtk_packet, query

NMEA_BAUDS = (4800, 9600, 14400, 19200, 38400, 57600, 115200)
NAV_THRESHOLDS = ("0", "0.2", "0.4", "0.6", "0.8", "1.0", "1.5", "2.0")
# PMTK314 field order: GLL RMC VTG GGA GSA GSV then reserved, ZDA(17) MCHN(18)
SENTENCES = ("gll", "rmc", "vtg", "gga", "gsa", "gsv", "zda", "mchn")
RESTARTS = {"hot": "PMTK101", "warm": "PMTK102", "cold": "PMTK103",
            "full-cold": "PMTK104"}
# PMTK514 value indexes for the settable sentences (mirrors PMTK314 layout)
NMEA_FIELDS = (("gll", 0), ("rmc", 1), ("vtg", 2), ("gga", 3), ("gsa", 4),
               ("gsv", 5), ("zda", 17), ("mchn", 18))


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


def find_fields(lines, prefix):
    """Split the first reply starting with prefix (sans *checksum)."""
    for r in lines:
        core = r.split("*")[0]
        if core == prefix or core.startswith(prefix + ","):
            return core.split(",")
    return None


def snap_scalar(lines, prefix, idx):
    """One field out of a query reply; None when the reply is missing."""
    f = find_fields(lines, prefix)
    if not f or len(f) <= idx:
        return None
    return f[idx]


def revert_datum(lines):
    """Rebuild a datum command; non-WGS84 snapshots need --confirm."""
    got = snap_scalar(lines, "$PMTK530", 1)
    if got is None:
        return None
    cmd = "datum --datum %s" % got
    return cmd if got == "0" else cmd + " --confirm"


def check_scalar(prefix, idx, want):
    """Verify closure: reply field must equal the value just set."""
    def check(lines):
        got = snap_scalar(lines, prefix, idx)
        if got is None:
            return False, "no %s reply" % prefix
        return got == want, "%s=%s (want %s)" % (prefix, got, want)
    return check


def revert_scalar(subcmd, prefix, idx, fmt=lambda v: v):
    """Revert closure: rebuild the subcommand from the snapshot value."""
    def revert(lines):
        got = snap_scalar(lines, prefix, idx)
        if got is None:
            return None
        return "%s %s" % (subcmd, fmt(got))
    return revert


def add_common(sub, needs_confirm=False, confirm_text=""):
    """Attach --dryrun/--verbose/--confirm flags shared by subcommands."""
    sub.add_argument("-v", "--verbose", action="store_true")
    sub.add_argument("--dryrun", action="store_true",
                     help="print packet without opening the port")
    sub.add_argument("--no-verify", action="store_true",
                     help="skip the post-change query re-read")
    if needs_confirm:
        sub.add_argument("--confirm", action="store_true",
                         help=confirm_text or "required to apply")


def snap_nmea_vals(lines):
    """19 PMTK514 values; None when the reply is missing or malformed."""
    f = find_fields(lines, "$PMTK514")
    if not f or len(f) < 20:
        return None
    vals = f[1:20]
    if not all(v.lstrip("-").isdigit() for v in vals):
        return None
    return vals


def check_nmea_out(want_vals):
    """Verify closure: None want (i.e. --defaults) accepts any sane reply."""
    def check(lines):
        got = snap_nmea_vals(lines)
        if got is None:
            return False, "no parseable $PMTK514 reply"
        if want_vals is None:
            return True, "chip reports [%s]" % ",".join(got)
        return got == want_vals, "PMTK514 [%s]" % ",".join(got)
    return check


def revert_nmea_out(lines):
    """Rebuild an nmea-out command from the snapshot values."""
    vals = snap_nmea_vals(lines)
    if vals is None:
        return None
    return "nmea-out " + " ".join("--%s %s" % (n, vals[i])
                                  for n, i in NMEA_FIELDS)


def check_nav_threshold(want):
    """Float-tolerant verify: chip echoes e.g. 0.40 for a 0.4 set."""
    def check(lines):
        got = snap_scalar(lines, "$PMTK527", 1)
        if got is None:
            return False, "no $PMTK527 reply"
        try:
            ok = abs(float(got) - float(want)) < 1e-9
        except ValueError:
            ok = False
        return ok, "$PMTK527=%s (want %s)" % (got, want)
    return check


def revert_sbas(lines):
    """Rebuild a sbas command from the enable + mode snapshot."""
    en = snap_scalar(lines, "$PMTK513", 1)
    if en is None:
        return None
    cmd = "sbas " + ("--enable" if en == "1" else "--disable")
    md = snap_scalar(lines, "$PMTK519", 1)
    if md is not None:
        cmd += " --mode " + ("test" if md == "0" else "integrity")
    return cmd


def send_checked(ser, body, verbose):
    """Send a set-command, wait for its ACK, fail closed unless flag=3."""
    cmd = body.split(",")[0].replace("PMTK", "")
    replies = query(ser, body, 3.0, verbose)
    flag = ack_flag(replies, cmd)
    meanings = {0: "invalid packet", 1: "unsupported on this firmware",
                2: "valid but action failed", 3: "ok"}
    if flag is None:
        print("no ACK for PMTK%s (replies: %s)" % (cmd, replies or "none"),
              file=sys.stderr)
        return 1
    print("PMTK%s: %s (flag=%d)" % (cmd, meanings.get(flag, "?"), flag))
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
    ms = int(round(1000 / args.hz))
    if not 100 <= ms <= 10000:
        raise ValueError("interval %d ms out of range 100-10000" % ms)
    if args.hz != 1:
        print("WARNING: EASY self-ephemeris only works at 1 Hz",
              file=sys.stderr)
    return "PMTK220,%d" % ms


def require_confirm(args, what):


    """Abort unless --confirm was passed."""
    if not args.confirm:
        print("refusing %s without --confirm" % what, file=sys.stderr)
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


def reread(ser, queries, verbose):
    """Collect query replies for snapshot / verify."""
    lines = []
    for qbody in queries:
        lines += query(ser, qbody, 2.0, verbose)
    return lines


def execute(args, bodies, queries=(), check=None, revert=None):
    """Snapshot, set (ACK-gated), verify. Returns process exit code."""
    if args.dryrun:
        for b in bodies:
            print(pmtk_packet(b).decode().strip())
        return 0
    ser = open_port(args.serial_port, args.baud)
    if ser is None:
        return 2
    try:
        snap = reread(ser, queries, args.verbose)
        if revert is not None:
            hint = revert(snap)
            if hint is not None:
                print("revert: %s %s %s" % (sys.argv[0], args.serial_port,
                                            hint), file=sys.stderr)
            else:
                print("revert: snapshot unreadable; re-run show before "
                      "retrying", file=sys.stderr)
        for b in bodies:
            if send_checked(ser, b, args.verbose) != 0:
                return 1
        if check is None or not queries:
            return 0
        if args.no_verify:
            print("verify: skipped (--no-verify)", file=sys.stderr)
            return 0
        ok, detail = check(reread(ser, queries, args.verbose))
        print("verify: %s (%s)" % ("MATCH" if ok else "MISMATCH", detail),
              file=sys.stderr)
        return 0 if ok else 1
    finally:
        ser.close()


def execute_baud(args, new_rate):
    """Baud switch: ACK-gate, then prove the link alive at the new rate."""
    body = "PMTK251,%d" % new_rate
    if args.dryrun:
        print(pmtk_packet(body).decode().strip())
        return 0
    ser = open_port(args.serial_port, args.baud)
    if ser is None:
        return 2
    try:
        snap_note = "revert: %s %s baud --baud-rate %d --confirm " \
            "(run with -b %d)" % (sys.argv[0], args.serial_port,
                                  args.baud, new_rate)
        if send_checked(ser, body, args.verbose) != 0:
            return 1
    finally:
        ser.close()
    print(snap_note, file=sys.stderr)
    if args.no_verify:
        print("verify: skipped (--no-verify)", file=sys.stderr)
        return 0
    time.sleep(0.3)
    try:
        new = serial.Serial(args.serial_port, new_rate, timeout=0.5)
    except Exception as exc:
        print("verify: MISMATCH (cannot reopen at %d: %s; chip IS at %d "
              "now -- fix the host side, then %s)"
              % (new_rate, exc, new_rate, snap_note), file=sys.stderr)
        return 1
    try:
        t0 = time.time()
        while time.time() - t0 < 4.0:
            line = new.readline()
            if not line:
                continue
            try:
                s = line.decode("ascii").strip()
            except UnicodeDecodeError:
                continue
            if s.startswith("$GP") or s.startswith("$GN") or \
                    s.startswith("$PMTK"):
                print("verify: MATCH (NMEA live at %d baud)" % new_rate,
                      file=sys.stderr)
                return 0
        print("verify: MISMATCH (silent at %d baud; chip accepted the "
              "switch -- check wiring, then %s)" % (new_rate, snap_note),
              file=sys.stderr)
        return 1
    finally:
        new.close()


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
    add_common(p)

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
        if args.dryrun:
            for _, body, _ in QUERIES:
                print(pmtk_packet(body).decode().strip())
            return 0
        ser = open_port(args.serial_port, args.baud)
        if ser is None:
            return 2
        try:
            for name, body, _ in QUERIES:
                for r in query(ser, body, 2.5, args.verbose):
                    print("  %-8s %s" % (name, r[:200]))
            return 0
        finally:
            ser.close()

    dgps_map = {"none": "0", "rtcm": "1", "waas": "2"}
    try:
        if args.cmd == "nmea-out":
            if args.defaults and any(getattr(args, s) is not None
                                     for s in SENTENCES):
                print("ERROR: --defaults conflicts with per-sentence "
                      "intervals", file=sys.stderr)
                return 2
            if args.defaults:
                return execute(args, ["PMTK314,-1"], ["PMTK414"],
                               check_nmea_out(None), revert_nmea_out)
            body = build_nmea_out(args)
            want = body.split(",", 1)[1].split(",")
            return execute(args, [body], ["PMTK414"],
                           check_nmea_out(want), revert_nmea_out)
        if args.cmd == "rate":
            # No query packet exists: ACK-gated only, previous rate
            # unknowable, so no snapshot either. Default is 1 Hz.
            print("revert: rate --hz 1 (previous rate is not queryable)",
                  file=sys.stderr)
            return execute(args, [build_rate(args)])
        if args.cmd == "baud":
            if not require_confirm(args, "baud change"):
                return 2
            return execute_baud(args, args.baud_rate)
        if args.cmd == "sbas":
            bodies = ["PMTK313,%d" % (1 if args.enable else 0)]
            queries = ["PMTK413"]
            want_en = "1" if args.enable else "0"
            if args.mode:
                bodies.append("PMTK319,%d" % (0 if args.mode == "test"
                                             else 1))
                queries.append("PMTK419")
            want_md = {"test": "0", "integrity": "1"}.get(args.mode)

            def check_sbas(lines):
                got = snap_scalar(lines, "$PMTK513", 1)
                if got is None:
                    return False, "no $PMTK513 reply"
                if got != want_en:
                    return False, "$PMTK513=%s (want %s)" % (got, want_en)
                if want_md is not None:
                    md = snap_scalar(lines, "$PMTK519", 1)
                    if md is None:
                        return False, "no $PMTK519 reply"
                    if md != want_md:
                        return False, "$PMTK519=%s (want %s)" % (md, want_md)
                    return True, "$PMTK513=%s $PMTK519=%s" % (got, md)
                return True, "$PMTK513=%s" % got
            return execute(args, bodies, queries, check_sbas, revert_sbas)
        if args.cmd == "dgps":
            want = dgps_map[args.mode]
            return execute(args, ["PMTK301,%s" % want], ["PMTK401"],
                           check_scalar("$PMTK501", 1, want),
                           revert_scalar("dgps --mode",
                                         "$PMTK501", 1,
                                         lambda v: {v2: k for k, v2 in
                                                    dgps_map.items()}.get(
                                             v, v)))
        if args.cmd == "datum":
            if not 0 <= args.datum <= 221:
                raise ValueError("datum must be 0-221")
            if args.datum != 0 and not require_confirm(args,
                                                       "non-WGS84 datum"):
                return 2
            want = str(args.datum)
            return execute(args, ["PMTK330,%s" % want], ["PMTK430"],
                           check_scalar("$PMTK530", 1, want),
                           revert_datum)
        if args.cmd == "nav-threshold":
            return execute(args, ["PMTK386,%s" % args.ms], ["PMTK447"],
                           check_nav_threshold(args.ms),
                           revert_scalar("nav-threshold --ms",
                                         "$PMTK527", 1, lambda v: v))
        if args.cmd == "qzss":
            if args.nmea is None and args.func is None:
                print("nothing to do: pass --nmea and/or --func")
                return 2
            # No documented query packets: ACK-gated only.
            print("revert: unknown (no QZSS query packet; re-run show)",
                  file=sys.stderr)
            bodies = []
            if args.nmea:
                bodies.append("PMTK351,%d"
                              % (1 if args.nmea == "enable" else 0))
            if args.func:
                # PMTK352 polarity is inverted: 0=enable, 1=disable
                bodies.append("PMTK352,%d"
                              % (0 if args.func == "enable" else 1))
            return execute(args, bodies)
        if args.cmd == "aic":
            # No documented query packet: ACK-gated only.
            print("revert: aic %s (no AIC query packet)"
                  % ("--disable" if args.enable else "--enable"),
                  file=sys.stderr)
            return execute(args, ["PMTK286,%d" % (1 if args.enable else 0)])
        if args.cmd == "easy":
            want = "1" if args.enable else "0"
            return execute(args, ["PMTK869,1,%s" % want], ["PMTK869,0"],
                           check_scalar("$PMTK869", 2, want),
                           revert_scalar("easy",
                                         "$PMTK869", 2,
                                         lambda v: "--enable" if v == "1"
                                         else "--disable"))
        if args.cmd == "locus":
            if args.interval is not None:
                if args.interval < 1:
                    raise ValueError("interval must be >= 1 second")
                want = str(args.interval)
                return execute(args, ["PMTK187,1,%s" % want], ["PMTK183"],
                               check_scalar("$PMTKLOG", 5, want),
                               revert_scalar("locus --interval",
                                             "$PMTKLOG", 5, lambda v: v))
            act = "start" if args.start else "stop"
            print("revert: locus --%s"
                  % ("stop" if args.start else "start"), file=sys.stderr)
            return execute(args, ["PMTK185,%d" % (0 if args.start else 1)])
        if args.cmd == "restart":
            if args.mode == "full-cold" and not require_confirm(
                    args, "full-cold (wipes config)"):
                return 2
            rc = execute(args, [RESTARTS[args.mode]])
            if rc == 0:
                print("chip restarting; re-run show to confirm fix",
                      file=sys.stderr)
            return rc
        if args.cmd == "standby":
            if not require_confirm(args, "standby"):
                return 2
            rc = execute(args, ["PMTK161,0"])
            if rc == 0:
                print("wake: reopen the port and send any byte",
                      file=sys.stderr)
            return rc
        parser.error("unknown command")
    except ValueError as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        return 2
    return 2  # unreachable; parser.error exits


if __name__ == "__main__":
    sys.exit(main())
