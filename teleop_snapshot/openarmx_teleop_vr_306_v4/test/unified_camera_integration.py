"""Read-only loopback test for the unified teleop DataChannel + RGB video."""

import argparse
import asyncio
import statistics
import time

import aiohttp
from aiortc import RTCPeerConnection


async def run(base_url, seconds):
    connector = aiohttp.TCPConnector(ssl=False)
    async with aiohttp.ClientSession(connector=connector) as session:
        websocket_url = base_url.replace('https://', 'wss://')
        websocket = await session.ws_connect(
            f'{websocket_url}/ws?client_id=loopback-unified-camera-test',
            ssl=False,
        )

        async def drain_websocket():
            async for _ in websocket:
                pass

        websocket_reader = asyncio.create_task(drain_websocket())
        peer = RTCPeerConnection()
        peer.createDataChannel('teleop', ordered=False, maxRetransmits=0)
        peer.addTransceiver('video', direction='recvonly')
        frame_times = []
        first_frame = asyncio.Event()
        stream_finished = asyncio.Event()
        stream_errors = []

        @peer.on('track')
        def on_track(track):
            if track.kind != 'video':
                return

            async def consume():
                deadline = None
                try:
                    while deadline is None or time.monotonic() < deadline:
                        await asyncio.wait_for(track.recv(), timeout=2.0)
                        received_at = time.monotonic()
                        if deadline is None:
                            deadline = received_at + seconds
                        frame_times.append(received_at)
                        first_frame.set()
                except Exception as exc:
                    stream_errors.append(
                        f'{type(exc).__name__}: {exc} after '
                        f'{len(frame_times)} frames'
                    )
                finally:
                    stream_finished.set()

            asyncio.create_task(consume())

        offer = await peer.createOffer()
        await peer.setLocalDescription(offer)
        async with session.post(
                f'{base_url}/api/realtime/offer',
                ssl=False,
                json={
                    'sdp': peer.localDescription.sdp,
                    'type': peer.localDescription.type,
                    'generation': 1,
                }) as response:
            answer = await response.json()
            if response.status != 200:
                raise RuntimeError(answer)
        if answer.get('camera_video') is not True:
            raise RuntimeError(f'camera track missing from answer: {answer}')
        from aiortc import RTCSessionDescription
        await peer.setRemoteDescription(
            RTCSessionDescription(sdp=answer['sdp'], type=answer['type'])
        )
        await asyncio.wait_for(first_frame.wait(), timeout=5.0)
        await asyncio.wait_for(stream_finished.wait(), timeout=seconds + 3.0)
        await peer.close()
        await websocket.close()
        await asyncio.gather(websocket_reader, return_exceptions=True)

    intervals = [
        later - earlier for earlier, later in zip(frame_times, frame_times[1:])
    ]
    duration = frame_times[-1] - frame_times[0]
    fps = (len(frame_times) - 1) / duration if duration > 0 else 0.0
    maximum_gap = max(intervals, default=0.0)
    p99_gap = (
        statistics.quantiles(intervals, n=100)[98]
        if len(intervals) >= 100 else maximum_gap
    )
    print(
        f'frames={len(frame_times)} fps={fps:.2f} '
        f'max_gap_ms={maximum_gap * 1000:.1f} '
        f'p99_gap_ms={p99_gap * 1000:.1f}'
    )
    if stream_errors:
        raise RuntimeError('; '.join(stream_errors))
    if duration < seconds * 0.85:
        raise RuntimeError(
            f'RGB video covered only {duration:.2f}s of {seconds:.2f}s'
        )
    if fps < 27.0:
        raise RuntimeError(f'RGB video rate is too low: {fps:.2f} FPS')
    if maximum_gap > 0.20:
        raise RuntimeError(
            f'RGB video stalled for {maximum_gap * 1000:.1f} ms'
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--base-url', default='https://127.0.0.1:8544')
    parser.add_argument('--seconds', type=float, default=10.0)
    args = parser.parse_args()
    asyncio.run(run(args.base_url, max(3.0, args.seconds)))


if __name__ == '__main__':
    main()
