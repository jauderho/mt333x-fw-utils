#!/usr/bin/env python3
# mt333x_probe.py - Read-only MTK3339 prober: NMEA census + PMTK queries.
#
# Read-only by design: sends documented query packets only (PMTK605/414/413/
# 401/607/447/430/419/869/183, PMTK622,1 framed LOCUS dump). Never sends
# restart (101-104), erase (184), baud/update changes (220/251), standby/
# periodic modes (161/225), or the boot-ROM handshake. Safe to run while
# debugging a live receiver; stop gpsd first so the port is free.
#
# Usage: mt333x_probe.py SERIAL_PORT [-b BAUD] [-t SECONDS] [--verbose]
#   --dryrun prints the query packets without opening the port.
"""Read-only prober for MediaTek MTK3339 GPS receivers."""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None  # --dryrun / --help work without pyserial

QUERIES = (
    ("release", "PMTK605", "firmware release (returns PMTK705)"),
    ("nmea-out", "PMTK414", "NMEA sentence rates (returns PMTK514)"),
    ("sbas", "PMTK413", "SBAS enabled (returns PMTK513)"),
    ("dgps", "PMTK401", "DGPS mode (returns PMTK501)"),
    ("epo", "PMTK607", "EPO aiding status (returns PMTK707)"),
    ("navthr", "PMTK447", "nav speed threshold (returns PMTK527)"),
    ("datum", "PMTK430", "datum in use (returns PMTK530)"),
    ("sbasmode", "PMTK419", "SBAS test/integrity mode (returns PMTK519)"),
    ("easy", "PMTK869,0", "EASY aiding enabled (returns PMTK869,2,x)"),
    ("locus", "PMTK183", "LOCUS logger status (returns PMTKLOG)"),
)


def pmtk_checksum(body):
    """XOR checksum over the packet body (between $ and *)."""
    cks = 0
    for ch in body:
        cks ^= ord(ch)
    return "%02X" % cks


def pmtk_packet(body):
    """Build a full $PMTK...*CK<CR><LF> packet."""
    return ("$" + body + "*" + pmtk_checksum(body) + "\r\n").encode("ascii")


def census(ser, seconds, verbose):
    """Count sentence types for `seconds`; return dict like {'GPGGA': n}."""
    kinds = {}
    t0 = time.time()
    while time.time() - t0 < seconds:
        line = ser.readline()
        if not line:
            continue
        try:
            s = line.decode("ascii").strip()
        except UnicodeDecodeError:
            continue
        if len(s) > 6 and s[0] == "$":
            key = s[1:6]
            kinds[key] = kinds.get(key, 0) + 1
            if verbose and key.startswith("PMTK"):
                print("  spont: " + s[:160])
    return kinds


def query(ser, body, wait, verbose):
    """Send one query packet; return non-NMEA PMTK replies seen."""
    ser.reset_input_buffer()
    ser.write(pmtk_packet(body))
    replies = []
    t0 = time.time()
    while time.time() - t0 < wait:
        line = ser.readline()
        if not line:
            continue
        try:
            s = line.decode("ascii").strip()
        except UnicodeDecodeError:
            continue
        if "PMTK" in s and "PMTKLOX" not in s:
            replies.append(s)
            if verbose:
                print("  rx: " + s[:200])
    return replies


def locus_dump(ser, wait=10.0, verbose=False):
    """Framed PMTK622,1 dump: returns (data_lines, start_n) without flooding.

    Reads PMTKLOX,0,n start, collects type-1 data lines, stops at PMTKLOX,2.
    Use PMTK185,1 beforehand only if you intend to stop logging (not done here).
    """
    ser.reset_input_buffer()
    ser.write(pmtk_packet("PMTK622,1"))
    data = []
    expect = None
    t0 = time.time()
    while time.time() - t0 < wait:
        line = ser.readline()
        if not line:
            continue
        try:
            s = line.decode("ascii").strip()
        except UnicodeDecodeError:
            continue
        if s.startswith("$PMTKLOX,0,"):
            try:
                expect = int(s.split(",")[2].split("*")[0])
            except ValueError:
                pass
        elif s.startswith("$PMTKLOX,1,"):
            data.append(s)
            if verbose and len(data) <= 3:
                print("  lox: " + s[:120])
        elif s.startswith("$PMTKLOX,2"):
            break
    return data, expect


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Read-only probe of a MediaTek MTK3339 GPS receiver.")
    parser.add_argument("serial_port", nargs="?",
                        help="serial device, e.g. /dev/ttyAMA0")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="NMEA baud rate (default: 115200)")
    parser.add_argument("-t", "--census-seconds", type=float, default=5.0,
                        help="seconds to census the stream (default: 5)")
    parser.add_argument("--locus-dump", action="store_true",
                        help="also run framed PMTK622,1 LOCUS dump")
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("--dryrun", action="store_true",
                        help="print query packets without opening the port")
    args = parser.parse_args(argv)

    if args.dryrun:
        for name, body, _ in QUERIES:
            print("%-8s %s" % (name, pmtk_packet(body).decode().strip()))
        print("%-8s %s" % ("locusdump", pmtk_packet("PMTK622,1").decode().strip()))
        return 0

    if not args.serial_port:
        parser.error("serial_port is required (unless --dryrun)")
    if serial is None:
        print("ERROR: pyserial is not installed", file=sys.stderr)
        return 1

    ser = serial.Serial(args.serial_port, args.baud, timeout=0.5)
    time.sleep(0.3)

    print("census (%.0fs @ %d baud)..." % (args.census_seconds, args.baud))
    kinds = census(ser, args.census_seconds, args.verbose)
    for key in sorted(kinds):
        print("  %-5s %d" % (key, kinds[key]))

    print("queries...")
    for name, body, desc in QUERIES:
        replies = query(ser, body, 2.5, args.verbose)
        if replies:
            for r in replies:
                print("  %-8s %s" % (name, r[:200]))
        else:
            print("  %-8s (no reply) -- %s" % (name, desc))

    if args.locus_dump:
        print("locus dump (PMTK622,1)...")
        data, expect = locus_dump(ser, verbose=args.verbose)
        print("  got %d data lines (announced %s)" % (len(data), expect))

    return 0


if __name__ == "__main__":
    sys.exit(main())
