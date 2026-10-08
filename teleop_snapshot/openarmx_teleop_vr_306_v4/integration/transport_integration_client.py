#!/usr/bin/env python3
"""Dry-run integration test for the 306 V4 WebRTC/WebSocket transport bridge.

The client intentionally sends no controller poses and no pressed buttons, so
the mapper cannot produce arm targets.  Run it only against a bridge launched
with ``dry_run:=true``.

It verifies:

* a generation-1 WebRTC data channel returns a matching monotonic ACK;
* generation 2 replaces generation 1 and stale generation offers are rejected;
* a retired generation cannot affect the active generation;
* WebRTC and WebSocket share one global sequence de-duplication boundary.

Dependencies (normally already present in robot_env): aiohttp, aiortc.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import ssl
import sys
import time
from dataclasses import dataclass, field
from typing import Any
from urllib.parse import urlparse

try:
    from aiohttp import ClientSession, TCPConnector, WSMsgType
    from aiortc import RTCPeerConnection, RTCSessionDescription
except ImportError as exc:  # Keep --help/use errors explicit on minimal hosts.
    raise SystemExit(
        "Missing integration-test dependency: "
        f"{exc}. Run with /home/ubuntu/ros2_ws/venvs/openarmx_v4_placo/bin/python."
    ) from exc


@dataclass
class PeerHandle:
    generation: int
    peer: RTCPeerConnection
    channel: Any
    open_event: asyncio.Event = field(default_factory=asyncio.Event)
    close_event: asyncio.Event = field(default_factory=asyncio.Event)
    ack_queue: asyncio.Queue[dict[str, Any]] = field(
        default_factory=asyncio.Queue
    )


def no_motion_packet(sequence: int) -> dict[str, Any]:
    """Return a valid state sample that cannot request robot motion."""
    released = {
        "position": None,
        "rotation": None,
        "quaternion": None,
        "buttons": {
            "grip": False,
            "trigger": 0.0,
            "a": False,
            "x": False,
        },
    }
    return {
        "packetType": "pose",
        "timestamp": int(time.time() * 1000),
        "sequence": int(sequence),
        "head": {
            "position": None,
            "rotation": None,
            "quaternion": None,
        },
        "leftController": dict(released),
        "rightController": dict(released),
        "integrationTest": True,
    }


def endpoint(base_url: str, *, websocket: bool, path: str) -> str:
    parsed = urlparse(base_url.rstrip("/"))
    if parsed.scheme not in {"http", "https"}:
        raise ValueError("--base-url must start with http:// or https://")
    scheme = (
        "wss" if websocket and parsed.scheme == "https"
        else "ws" if websocket
        else parsed.scheme
    )
    return parsed._replace(scheme=scheme, path=path, params="", query="", fragment="").geturl()


async def wait_ice_gathering_complete(
    peer: RTCPeerConnection, timeout: float
) -> None:
    if peer.iceGatheringState == "complete":
        return
    complete = asyncio.Event()

    @peer.on("icegatheringstatechange")
    def on_state_change() -> None:
        if peer.iceGatheringState == "complete":
            complete.set()

    await asyncio.wait_for(complete.wait(), timeout=timeout)


async def negotiate_peer(
    session: ClientSession,
    offer_url: str,
    generation: int,
    timeout: float,
) -> PeerHandle:
    peer = RTCPeerConnection(configuration=None)
    channel = peer.createDataChannel(
        "teleop", ordered=False, maxRetransmits=0
    )
    handle = PeerHandle(generation, peer, channel)

    @channel.on("open")
    def on_open() -> None:
        handle.open_event.set()

    @channel.on("close")
    def on_close() -> None:
        handle.close_event.set()

    @channel.on("message")
    def on_message(raw_message: Any) -> None:
        try:
            packet = json.loads(raw_message)
        except (TypeError, ValueError):
            return
        if packet.get("type") == "teleop_ack":
            handle.ack_queue.put_nowait(packet)

    offer = await peer.createOffer()
    await peer.setLocalDescription(offer)
    await wait_ice_gathering_complete(peer, timeout)
    payload = {
        "sdp": peer.localDescription.sdp,
        "type": peer.localDescription.type,
        "generation": generation,
    }
    async with session.post(offer_url, json=payload) as response:
        body = await response.text()
        if response.status != 200:
            await peer.close()
            raise AssertionError(
                f"generation {generation} offer failed: HTTP "
                f"{response.status}: {body}"
            )
        answer = json.loads(body)
    if int(answer.get("generation", -1)) != generation:
        await peer.close()
        raise AssertionError(
            f"answer generation mismatch: expected {generation}, got {answer}"
        )
    await peer.setRemoteDescription(
        RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
    )
    await asyncio.wait_for(handle.open_event.wait(), timeout=timeout)
    return handle


async def expect_ack(
    handle: PeerHandle, sequence: int, timeout: float
) -> dict[str, Any]:
    deadline = asyncio.get_running_loop().time() + timeout
    while True:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
            raise AssertionError(
                f"generation {handle.generation} did not ACK sequence {sequence}"
            )
        ack = await asyncio.wait_for(handle.ack_queue.get(), timeout=remaining)
        if (
            int(ack.get("generation", -1)) == handle.generation
            and int(ack.get("sequence", -1)) == sequence
        ):
            return ack


async def get_status(session: ClientSession, status_url: str) -> dict[str, Any]:
    async with session.get(status_url) as response:
        body = await response.text()
        if response.status != 200:
            raise AssertionError(
                f"status request failed: HTTP {response.status}: {body}"
            )
        return json.loads(body)


async def drain_websocket(websocket: Any) -> None:
    """Keep aiohttp reading so ping/pong control frames remain serviced."""
    async for message in websocket:
        if message.type in {
            WSMsgType.CLOSE,
            WSMsgType.CLOSED,
            WSMsgType.ERROR,
        }:
            return


def counter(status: dict[str, Any], group: str, transport: str) -> int:
    return int(status.get(group, {}).get(transport, 0))


async def run_test(args: argparse.Namespace) -> None:
    base_url = args.base_url.rstrip("/")
    websocket_url = endpoint(base_url, websocket=True, path="/ws")
    offer_url = endpoint(base_url, websocket=False, path="/api/realtime/offer")
    status_url = endpoint(base_url, websocket=False, path="/api/status")

    parsed = urlparse(base_url)
    if parsed.hostname not in {"127.0.0.1", "localhost", "::1"} and not args.allow_remote:
        raise SystemExit(
            "Refusing a non-loopback bridge. Run this on the robot against "
            "https://127.0.0.1:8443, or explicitly pass --allow-remote."
        )

    tls_context = ssl.create_default_context()
    if args.insecure:
        tls_context.check_hostname = False
        tls_context.verify_mode = ssl.CERT_NONE
    connector = TCPConnector(ssl=tls_context)
    peers: list[PeerHandle] = []

    async with ClientSession(connector=connector) as session:
        initial_status = await get_status(session, status_url)
        if initial_status.get("system") != "openarmx_teleop_vr_306_v4":
            raise AssertionError(
                f"unexpected bridge identity: {initial_status.get('system')!r}"
            )

        websocket = await session.ws_connect(
            websocket_url, heartbeat=5.0, timeout=args.timeout
        )
        websocket_reader = asyncio.create_task(drain_websocket(websocket))
        try:
            baseline = await get_status(session, status_url)
            if not baseline.get("vrConnected"):
                raise AssertionError("WebSocket ownership was not registered")

            first = await negotiate_peer(
                session, offer_url, generation=1, timeout=args.timeout
            )
            peers.append(first)
            first.channel.send(json.dumps(no_motion_packet(10), separators=(",", ":")))
            await expect_ack(first, 10, args.timeout)
            print("PASS generation 1: WebRTC data channel ACKed sequence 10")

            # ACKs are deliberately rate-limited by the bridge.
            await asyncio.sleep(0.15)
            second = await negotiate_peer(
                session, offer_url, generation=2, timeout=args.timeout
            )
            peers.append(second)

            # The old SCTP association should be closed. If the local close
            # notification has not arrived yet, deliberately send a very large
            # stale sequence. The server-side peer/generation identity guards
            # must ignore it, otherwise sequence 20 below would be rejected.
            stale_probe_sent = False
            if first.channel.readyState == "open":
                try:
                    first.channel.send(
                        json.dumps(
                            no_motion_packet(900_000), separators=(",", ":")
                        )
                    )
                    stale_probe_sent = True
                    await asyncio.sleep(0.12)
                except Exception:
                    # A concurrently delivered close notification is also the
                    # expected replacement behavior.
                    pass
            second.channel.send(json.dumps(no_motion_packet(20), separators=(",", ":")))
            await expect_ack(second, 20, args.timeout)
            print(
                "PASS generation 2: replacement ACKed sequence 20; "
                + ("stale generation packet was isolated" if stale_probe_sent else "old channel was retired")
            )

            # The offer generation itself is monotonic too. Reuse the already
            # established generation-1 SDP; rejection happens before SDP use.
            stale_payload = {
                "sdp": first.peer.localDescription.sdp,
                "type": first.peer.localDescription.type,
                "generation": 1,
            }
            async with session.post(offer_url, json=stale_payload) as response:
                if response.status != 409:
                    raise AssertionError(
                        "stale generation offer was not rejected: "
                        f"HTTP {response.status}: {await response.text()}"
                    )
            print("PASS generation ordering: stale offer rejected with HTTP 409")

            before_fallback = await get_status(session, status_url)
            await websocket.send_str(
                json.dumps(no_motion_packet(21), separators=(",", ":"))
            )
            await websocket.send_str(
                json.dumps(no_motion_packet(20), separators=(",", ":"))
            )
            await asyncio.sleep(0.20)
            after_fallback = await get_status(session, status_url)
            ws_accept_delta = (
                counter(after_fallback, "realtimePacketCounts", "websocket")
                - counter(before_fallback, "realtimePacketCounts", "websocket")
            )
            ws_drop_delta = (
                counter(after_fallback, "realtimeDropCounts", "websocket")
                - counter(before_fallback, "realtimeDropCounts", "websocket")
            )
            if ws_accept_delta != 1 or ws_drop_delta < 1:
                raise AssertionError(
                    "global sequence de-duplication failed: "
                    f"accepted delta={ws_accept_delta}, dropped delta={ws_drop_delta}, "
                    f"status={after_fallback}"
                )
            if after_fallback.get("realtimeTransport") != "websocket":
                raise AssertionError("accepted WS fallback did not become latest transport")
            print("PASS WS fallback: sequence 21 accepted and stale sequence 20 dropped")

            await asyncio.sleep(0.15)
            second.channel.send(json.dumps(no_motion_packet(22), separators=(",", ":")))
            await expect_ack(second, 22, args.timeout)
            await asyncio.sleep(0.05)
            final_status = await get_status(session, status_url)
            if final_status.get("realtimeTransport") != "webrtc":
                raise AssertionError("WebRTC did not resume after WS fallback")
            if float(final_status.get("realtimePacketAge", 99.0)) > 1.0:
                raise AssertionError(f"latest packet age is unexpectedly stale: {final_status}")
            print("PASS recovery: active WebRTC generation resumed at sequence 22")
        finally:
            for peer_handle in reversed(peers):
                await peer_handle.peer.close()
            await websocket.close()
            websocket_reader.cancel()
            await asyncio.gather(websocket_reader, return_exceptions=True)

    print("ALL TRANSPORT INTEGRATION CHECKS PASSED (no motion targets sent)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-url",
        default="https://127.0.0.1:8443",
        help="dry-run bridge base URL (default: %(default)s)",
    )
    parser.add_argument(
        "--timeout", type=float, default=10.0,
        help="timeout for each negotiation/ACK step",
    )
    parser.add_argument(
        "--insecure", action=argparse.BooleanOptionalAction, default=True,
        help="accept the bridge self-signed TLS certificate (default: true)",
    )
    parser.add_argument(
        "--allow-remote", action="store_true",
        help="allow a non-loopback target; still sends no controller poses",
    )
    return parser.parse_args()


def main() -> int:
    try:
        asyncio.run(run_test(parse_args()))
    except (AssertionError, asyncio.TimeoutError, OSError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
