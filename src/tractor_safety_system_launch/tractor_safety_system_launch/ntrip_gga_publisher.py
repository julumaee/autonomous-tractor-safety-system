#!/usr/bin/env python3

import datetime
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus


def _checksum_nmea(sentence_no_checksum: str) -> int:
    if not sentence_no_checksum.startswith("$"):
        raise ValueError("NMEA sentence must start with '$'")
    checksum = 0
    for character in sentence_no_checksum[1:]:
        checksum ^= ord(character)
    return checksum


def _dd_to_dmm_lat(lat_dd: float) -> Tuple[str, str]:
    direction = "N" if lat_dd >= 0.0 else "S"
    lat_abs = abs(lat_dd)
    degrees = int(lat_abs)
    minutes = (lat_abs - degrees) * 60.0
    return f"{degrees:02d}{minutes:07.4f}", direction


def _dd_to_dmm_lon(lon_dd: float) -> Tuple[str, str]:
    direction = "E" if lon_dd >= 0.0 else "W"
    lon_abs = abs(lon_dd)
    degrees = int(lon_abs)
    minutes = (lon_abs - degrees) * 60.0
    return f"{degrees:03d}{minutes:07.4f}", direction


class NtripGgaPublisher(Node):
    def __init__(self) -> None:
        super().__init__("ntrip_gga_publisher")

        self.declare_parameter("input_fix_topic", "/gps/fix")
        self.declare_parameter("output_nmea_topic", "/ntrip/nmea")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("require_valid_fix_status", False)
        self.declare_parameter("reject_zero_latlon", True)
        self.declare_parameter("talker", "GP")
        self.declare_parameter("quality_override", -1)
        self.declare_parameter("quality_when_no_fix", 0)
        self.declare_parameter("sats", 8)
        self.declare_parameter("hdop", 1.0)

        input_fix_topic = self.get_parameter("input_fix_topic").get_parameter_value().string_value
        output_nmea_topic = (
            self.get_parameter("output_nmea_topic").get_parameter_value().string_value
        )
        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self._require_valid_fix_status = (
            self.get_parameter("require_valid_fix_status").get_parameter_value().bool_value
        )
        self._reject_zero_latlon = (
            self.get_parameter("reject_zero_latlon").get_parameter_value().bool_value
        )
        self._talker = self.get_parameter("talker").get_parameter_value().string_value or "GP"
        self._quality_override = int(
            self.get_parameter("quality_override").get_parameter_value().integer_value
        )
        self._quality_when_no_fix = int(
            self.get_parameter("quality_when_no_fix").get_parameter_value().integer_value
        )
        self._sats = int(self.get_parameter("sats").get_parameter_value().integer_value)
        self._hdop = float(self.get_parameter("hdop").get_parameter_value().double_value)

        if self._sats < 0 or self._sats > 99:
            raise ValueError("sats must be in range [0, 99]")
        if self._quality_override != -1 and (
            self._quality_override < 0 or self._quality_override > 8
        ):
            raise ValueError("quality_override must be -1 or in range [0, 8]")
        if self._quality_when_no_fix < 0 or self._quality_when_no_fix > 8:
            raise ValueError("quality_when_no_fix must be in range [0, 8]")

        if publish_hz <= 0.0:
            raise ValueError("publish_hz must be > 0")

        self._last_fix: Optional[NavSatFix] = None

        self._pub = self.create_publisher(Sentence, output_nmea_topic, 10)
        self._sub = self.create_subscription(NavSatFix, input_fix_topic, self._on_fix, 10)
        self._timer = self.create_timer(1.0 / publish_hz, self._on_timer)

        self.get_logger().info(
            f"Publishing {self._talker}GGA on '{output_nmea_topic}' at {publish_hz} Hz "
            f"from '{input_fix_topic}'"
        )

    def _on_fix(self, msg: NavSatFix) -> None:
        self._last_fix = msg

    def _on_timer(self) -> None:
        fix = self._last_fix
        if fix is None:
            return

        if self._require_valid_fix_status and fix.status.status == NavSatStatus.STATUS_NO_FIX:
            return

        if not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
            return

        if self._reject_zero_latlon and fix.latitude == 0.0 and fix.longitude == 0.0:
            return

        # NMEA UTC time: hhmmss.ss using current UTC time.
        now = datetime.datetime.now(datetime.UTC)
        utc_str = now.strftime("%H%M%S") + f".{int(now.microsecond / 10000):02d}"

        lat_dmm, lat_dir = _dd_to_dmm_lat(fix.latitude)
        lon_dmm, lon_dir = _dd_to_dmm_lon(fix.longitude)

        if self._quality_override != -1:
            quality = self._quality_override
        else:
            status = fix.status.status
            if status == NavSatStatus.STATUS_FIX:
                quality = 1
            elif status == NavSatStatus.STATUS_SBAS_FIX:
                quality = 2
            elif status == NavSatStatus.STATUS_GBAS_FIX:
                quality = 5
            elif status == NavSatStatus.STATUS_NO_FIX:
                quality = self._quality_when_no_fix
            else:
                quality = 0

        sats = f"{self._sats:02d}"
        hdop = f"{self._hdop:.1f}"
        altitude = fix.altitude if math.isfinite(fix.altitude) else 0.0

        sentence_no_checksum = (
            f"${self._talker}GGA,{utc_str},{lat_dmm},{lat_dir},"
            f"{lon_dmm},{lon_dir},{quality},{sats},{hdop},{altitude:.1f},M,0.0,M,,"
        )
        checksum = _checksum_nmea(sentence_no_checksum)
        sentence = f"{sentence_no_checksum}*{checksum:02X}\r\n"

        msg = Sentence()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = fix.header.frame_id or "gps"
        msg.sentence = sentence
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = NtripGgaPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
