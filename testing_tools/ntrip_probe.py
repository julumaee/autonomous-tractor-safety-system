#!/usr/bin/env python3

import argparse
import base64
import datetime
import socket
import ssl
import sys
import time
from typing import Optional, Tuple


def dd_to_dmm_lat(lat_dd: float) -> Tuple[str, str]:
    direction = "N" if lat_dd >= 0.0 else "S"
    lat_abs = abs(lat_dd)
    degrees = int(lat_abs)
    minutes = (lat_abs - degrees) * 60.0
    return f"{degrees:02d}{minutes:07.4f}", direction


def dd_to_dmm_lon(lon_dd: float) -> Tuple[str, str]:
    direction = "E" if lon_dd >= 0.0 else "W"
    lon_abs = abs(lon_dd)
    degrees = int(lon_abs)
    minutes = (lon_abs - degrees) * 60.0
    return f"{degrees:03d}{minutes:07.4f}", direction


def nmea_checksum(sentence_no_checksum: str) -> int:
    if not sentence_no_checksum.startswith("$"):
        raise ValueError("NMEA sentence must start with '$'")
    checksum = 0
    for character in sentence_no_checksum[1:]:
        checksum ^= ord(character)
    return checksum


def make_gga(
    talker: str,
    lat_dd: float,
    lon_dd: float,
    altitude_m: float,
    quality: int,
    sats: int,
    hdop: float,
) -> str:
    now = datetime.datetime.now(datetime.UTC)
    utc_str = now.strftime("%H%M%S") + f".{int(now.microsecond / 10000):02d}"

    lat_dmm, lat_dir = dd_to_dmm_lat(lat_dd)
    lon_dmm, lon_dir = dd_to_dmm_lon(lon_dd)

    sentence_no_checksum = (
        f"${talker}GGA,{utc_str},{lat_dmm},{lat_dir},"
        f"{lon_dmm},{lon_dir},{quality},{sats:02d},{hdop:.1f},{altitude_m:.1f},M,0.0,M,,"
    )
    checksum = nmea_checksum(sentence_no_checksum)
    return f"{sentence_no_checksum}*{checksum:02X}\r\n"


def read_http_response(sock: socket.socket, timeout_s: float) -> str:
    sock.settimeout(timeout_s)
    buf = b""
    while b"\r\n\r\n" not in buf and len(buf) < 64 * 1024:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buf += chunk
    try:
        return buf.decode("ISO-8859-1", errors="replace")
    except Exception:
        return repr(buf)


def form_request(
    host: str,
    port: int,
    mountpoint: str,
    ntrip_version: Optional[str],
    username: Optional[str],
    password: Optional[str],
    user_agent: str,
    add_host_header: bool,
) -> bytes:
    if mountpoint.startswith("/"):
        mountpoint = mountpoint[1:]

    lines = [f"GET /{mountpoint} HTTP/1.0"]
    if add_host_header:
        lines.append(f"Host: {host}:{port}")
    if ntrip_version:
        lines.append(f"Ntrip-Version: {ntrip_version}")
    lines.append(f"User-Agent: {user_agent}")

    if username is not None and password is not None:
        creds = base64.b64encode(f"{username}:{password}".encode("utf-8")).decode("utf-8")
        lines.append(f"Authorization: Basic {creds}")

    lines.append("")
    lines.append("")
    return "\r\n".join(lines).encode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe an NTRIP caster outside ROS and show the server response / stability.")
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=2101)
    parser.add_argument("--mountpoint", required=True)
    parser.add_argument("--ntrip-version", default="Ntrip/2.0")
    parser.add_argument("--authenticate", action="store_true")
    parser.add_argument("--username", default="")
    parser.add_argument("--password", default="")
    parser.add_argument("--ssl", action="store_true")

    parser.add_argument("--user-agent", default="NTRIP ntrip_probe")
    parser.add_argument("--add-host-header", action="store_true", help="Include a Host: header (some casters are picky)")

    parser.add_argument("--send-gga", action="store_true", help="Send GGA once per second")
    parser.add_argument("--talker", default="GP", help="Talker ID (e.g., GP or GN)")
    parser.add_argument("--lat", type=float, default=60.1699)
    parser.add_argument("--lon", type=float, default=24.9384)
    parser.add_argument("--alt", type=float, default=10.0)
    parser.add_argument("--quality", type=int, default=1)
    parser.add_argument("--sats", type=int, default=8)
    parser.add_argument("--hdop", type=float, default=1.0)

    parser.add_argument("--duration", type=float, default=15.0, help="Seconds to keep the session open")
    parser.add_argument("--timeout", type=float, default=5.0, help="Socket recv timeout")
    parser.add_argument("--verbose", action="store_true")

    args = parser.parse_args()

    username: Optional[str] = None
    password: Optional[str] = None
    if args.authenticate:
        if not args.username or not args.password:
            print("ERROR: --authenticate requires non-empty --username and --password", file=sys.stderr)
            return 2
        username = args.username
        password = args.password

    request = form_request(
        host=args.host,
        port=args.port,
        mountpoint=args.mountpoint,
        ntrip_version=None if args.ntrip_version in ("", "None", "none") else args.ntrip_version,
        username=username,
        password=password,
        user_agent=args.user_agent,
        add_host_header=args.add_host_header,
    )

    raw = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    raw.settimeout(args.timeout)

    try:
        raw.connect((args.host, args.port))
    except Exception as e:
        print(f"ERROR: connect failed: {e}", file=sys.stderr)
        return 1

    sock: socket.socket
    if args.ssl:
        ctx = ssl.create_default_context()
        sock = ctx.wrap_socket(raw, server_hostname=args.host)
    else:
        sock = raw

    try:
        if args.verbose:
            print("--- Request ---")
            print(request.decode("utf-8", errors="replace"))
        sock.sendall(request)
        response = read_http_response(sock, timeout_s=args.timeout)
        print("--- Response ---")
        print(response.strip())

        start = time.time()
        next_gga = start
        total_bytes = 0
        packets = 0

        while time.time() - start < args.duration:
            if args.send_gga and time.time() >= next_gga:
                gga = make_gga(
                    talker=args.talker,
                    lat_dd=args.lat,
                    lon_dd=args.lon,
                    altitude_m=args.alt,
                    quality=args.quality,
                    sats=args.sats,
                    hdop=args.hdop,
                )
                if args.verbose:
                    print(f"--- GGA -> ({len(gga)} bytes) ---")
                    print(gga.strip())
                sock.sendall(gga.encode("utf-8"))
                next_gga += 1.0

            try:
                data = sock.recv(4096)
                if not data:
                    print("--- Socket closed by peer ---")
                    break
                total_bytes += len(data)
                packets += 1
                head = data[:24].hex(" ")
                print(f"RTCM bytes: +{len(data)} (total={total_bytes}, packets={packets}) head={head}")
            except socket.timeout:
                pass

            time.sleep(0.05)

        return 0
    except Exception as e:
        print(f"ERROR: session error: {e}", file=sys.stderr)
        return 1
    finally:
        try:
            sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
