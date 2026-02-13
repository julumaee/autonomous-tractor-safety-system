#!/usr/bin/env python3

import base64
import select
import socket
import ssl
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nmea_msgs.msg import Sentence


@dataclass
class _RtcmFrame:
    data: bytes


def _iter_rtcm3_frames(buffer: bytearray):
    # RTCM3 framing:
    # - preamble: 0xD3
    # - 10-bit length in next 2 bytes (lower 2 bits of byte1 + byte2)
    # - payload: length bytes
    # - CRC24Q: 3 bytes
    while True:
        if len(buffer) < 6:
            return

        if buffer[0] != 0xD3:
            idx = buffer.find(b"\xD3")
            if idx == -1:
                buffer.clear()
                return
            del buffer[:idx]
            if len(buffer) < 6:
                return

        length = ((buffer[1] & 0x03) << 8) | buffer[2]
        total_len = 3 + length + 3
        if total_len < 6:
            del buffer[0]
            continue
        if len(buffer) < total_len:
            return

        frame = bytes(buffer[:total_len])
        del buffer[:total_len]
        yield _RtcmFrame(frame)


def _read_http_headers(
    sock: socket.socket, timeout_s: float = 5.0, max_bytes: int = 64 * 1024
) -> bytes:
    sock.settimeout(timeout_s)
    buf = bytearray()
    while b"\r\n\r\n" not in buf and len(buf) < max_bytes:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buf.extend(chunk)
    return bytes(buf)


def _form_request(
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
        creds = base64.b64encode(
            f"{username}:{password}".encode("utf-8")
        ).decode("utf-8")
        lines.append(f"Authorization: Basic {creds}")

    lines.append("")
    lines.append("")
    return "\r\n".join(lines).encode("utf-8")


class NtripTcpClient(Node):
    def __init__(self) -> None:
        super().__init__("ntrip_client")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 2101)
        self.declare_parameter("mountpoint", "mount")
        self.declare_parameter("ntrip_version", "Ntrip/2.0")
        self.declare_parameter("authenticate", False)
        self.declare_parameter("username", "")
        self.declare_parameter("password", "")

        self.declare_parameter("ssl", False)

        # Keep parameter parity with the upstream ntrip_client node/launch.
        # Determines which RTCM message type we publish on the 'rtcm' topic.
        self.declare_parameter("rtcm_message_package", "rtcm_msgs")

        # Critical for RTK2go: the system ntrip_client UA triggers a sourcetable response.
        self.declare_parameter("user_agent", "NTRIP tractor_safety_system")
        self.declare_parameter("add_host_header", True)

        self.declare_parameter("rtcm_frame_id", "odom")
        self.declare_parameter("reconnect_wait_seconds", 2.0)

        host = self.get_parameter("host").get_parameter_value().string_value
        port = int(self.get_parameter("port").get_parameter_value().integer_value)
        mountpoint = self.get_parameter("mountpoint").get_parameter_value().string_value
        ntrip_version = self.get_parameter("ntrip_version").get_parameter_value().string_value
        authenticate = self.get_parameter("authenticate").get_parameter_value().bool_value
        username = self.get_parameter("username").get_parameter_value().string_value
        password = self.get_parameter("password").get_parameter_value().string_value

        self._ssl = self.get_parameter("ssl").get_parameter_value().bool_value
        self._rtcm_message_package = (
            self.get_parameter("rtcm_message_package").get_parameter_value().string_value
            or "rtcm_msgs"
        )
        self._user_agent = self.get_parameter("user_agent").get_parameter_value().string_value
        self._add_host_header = (
            self.get_parameter("add_host_header").get_parameter_value().bool_value
        )
        self._rtcm_frame_id = (
            self.get_parameter("rtcm_frame_id").get_parameter_value().string_value
        )
        self._reconnect_wait_seconds = float(
            self.get_parameter("reconnect_wait_seconds").get_parameter_value().double_value
        )

        if not host:
            raise ValueError("host must be non-empty")
        if port <= 0 or port > 65535:
            raise ValueError("port must be in range [1, 65535]")
        if not mountpoint:
            raise ValueError("mountpoint must be non-empty")

        if ntrip_version in ("", "None", "none"):
            self._ntrip_version: Optional[str] = None
        else:
            self._ntrip_version = ntrip_version

        if authenticate:
            if not username or not password:
                raise ValueError("authenticate:=true requires non-empty username and password")
            self._username: Optional[str] = username
            self._password: Optional[str] = password
        else:
            self._username = None
            self._password = None

        self._host = host
        self._port = port
        self._mountpoint = mountpoint

        # Import RTCM message types lazily.
        self._rtcm_msg_type = None
        self._create_rtcm_message = None

        self._setup_rtcm_message_type()
        self._rtcm_pub = self.create_publisher(self._rtcm_msg_type, "rtcm", 10)
        self._nmea_sub = self.create_subscription(Sentence, "nmea", self._on_nmea, 10)

        self._sock: Optional[socket.socket] = None
        self._recv_buffer = bytearray()
        self._connected = False
        self._next_connect_time = 0.0

        self._timer = self.create_timer(0.05, self._tick)

    def _setup_rtcm_message_type(self) -> None:
        prefer = self._rtcm_message_package.strip() if self._rtcm_message_package else "rtcm_msgs"

        def _try_rtcm_msgs() -> bool:
            try:
                from rtcm_msgs.msg import Message as RtcmMsg

                self._rtcm_msg_type = RtcmMsg

                def _mk(rtcm_bytes: bytes):
                    msg = RtcmMsg()
                    msg.header = Header(
                        stamp=self.get_clock().now().to_msg(), frame_id=self._rtcm_frame_id
                    )
                    msg.message = list(rtcm_bytes)
                    return msg

                self._create_rtcm_message = _mk
                return True
            except Exception:
                return False

        def _try_mavros_msgs() -> bool:
            try:
                from mavros_msgs.msg import RTCM as MavrosRtcm

                self._rtcm_msg_type = MavrosRtcm

                def _mk(rtcm_bytes: bytes):
                    msg = MavrosRtcm()
                    msg.header = Header(
                        stamp=self.get_clock().now().to_msg(), frame_id=self._rtcm_frame_id
                    )
                    msg.data = list(rtcm_bytes)
                    return msg

                self._create_rtcm_message = _mk
                return True
            except Exception:
                return False

        if prefer == "mavros_msgs":
            if _try_mavros_msgs() or _try_rtcm_msgs():
                return
        else:
            if _try_rtcm_msgs() or _try_mavros_msgs():
                return

        raise RuntimeError(
            "Neither rtcm_msgs nor mavros_msgs is available; install "
            "ros-jazzy-rtcm-msgs or ros-jazzy-mavros-msgs"
        )

    def _disconnect(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._connected = False
        self._recv_buffer.clear()

    def _connect(self) -> bool:
        self._disconnect()

        raw = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        raw.settimeout(5)

        try:
            raw.connect((self._host, self._port))
        except Exception as e:
            self.get_logger().warning(f"Connect failed: {e}")
            return False

        sock: socket.socket
        if self._ssl:
            ctx = ssl.create_default_context()
            sock = ctx.wrap_socket(raw, server_hostname=self._host)
        else:
            sock = raw

        request = _form_request(
            host=self._host,
            port=self._port,
            mountpoint=self._mountpoint,
            ntrip_version=self._ntrip_version,
            username=self._username,
            password=self._password,
            user_agent=self._user_agent,
            add_host_header=self._add_host_header,
        )

        try:
            sock.sendall(request)
            resp = _read_http_headers(sock)
        except Exception as e:
            self.get_logger().warning(f"Handshake failed: {e}")
            try:
                sock.close()
            except Exception:
                pass
            return False

        resp_text = resp.decode("ISO-8859-1", errors="replace")
        header_part = resp_text.split("\r\n\r\n", 1)[0].lower()

        if "gnss/sourcetable" in header_part or "sourcetable" in header_part:
            self.get_logger().error(
                "Caster returned a SOURCETABLE instead of RTCM data stream. "
                "This often happens with incompatible User-Agent/headers."
            )
            self.get_logger().error("Response headers: " + header_part.replace("\r\n", " | "))
            try:
                sock.close()
            except Exception:
                pass
            return False

        if "200" not in header_part and "icy" not in header_part:
            self.get_logger().error("Invalid response from caster")
            self.get_logger().error(resp_text)
            try:
                sock.close()
            except Exception:
                pass
            return False

        sock.setblocking(False)
        self._sock = sock
        self._connected = True

        self.get_logger().info(f"Connected to http://{self._host}:{self._port}/{self._mountpoint}")
        return True

    def _on_nmea(self, msg: Sentence) -> None:
        if not self._connected or self._sock is None:
            return

        sentence = msg.sentence
        if sentence.endswith("\\r\\n"):
            sentence = sentence[:-4] + "\r\n"
        elif not sentence.endswith("\r\n"):
            sentence = sentence + "\r\n"

        try:
            self._sock.sendall(sentence.encode("utf-8"))
        except Exception as e:
            self.get_logger().warning(f"Unable to send NMEA: {e}")
            self._disconnect()
            self._next_connect_time = time.time() + self._reconnect_wait_seconds

    def _tick(self) -> None:
        now = time.time()
        if not self._connected:
            if now < self._next_connect_time:
                return
            if not self._connect():
                self._next_connect_time = now + self._reconnect_wait_seconds
            return

        if self._sock is None:
            self._disconnect()
            self._next_connect_time = now + self._reconnect_wait_seconds
            return

        try:
            readable, _, _ = select.select([self._sock], [], [], 0.0)
        except Exception:
            self._disconnect()
            self._next_connect_time = now + self._reconnect_wait_seconds
            return

        if not readable:
            return

        try:
            chunk = self._sock.recv(4096)
        except BlockingIOError:
            return
        except Exception as e:
            self.get_logger().warning(f"Socket read error: {e}")
            self._disconnect()
            self._next_connect_time = now + self._reconnect_wait_seconds
            return

        if not chunk:
            self.get_logger().warning("Socket closed by peer")
            self._disconnect()
            self._next_connect_time = now + self._reconnect_wait_seconds
            return

        self._recv_buffer.extend(chunk)

        for frame in _iter_rtcm3_frames(self._recv_buffer):
            self._rtcm_pub.publish(self._create_rtcm_message(frame.data))


def main() -> None:
    rclpy.init()
    node = NtripTcpClient()
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
