import asyncio
import json
import ssl
import threading
from pathlib import Path
import websockets


def create_ssl_context(cert_file: str, key_file: str):
    """Create SSL context for WSS (secure WebSocket)."""
    ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ssl_context.load_cert_chain(cert_file, key_file)
    return ssl_context


class VRHeadset:
    name = "vr_headset"

    def __init__(self, use_ssl: bool = True, cert_file: str = None, key_file: str = None):
        self.connected = False
        self.last_observation = None
        self._server = None
        self._loop = None
        self._thread = None
        self._ready_event = threading.Event()  # to signal readiness
        
        # SSL configuration
        self.use_ssl = use_ssl
        self.ssl_context = None
        
        # Default certificate paths (relative to project root)
        if cert_file is None:
            cert_file = Path(__file__).parent.parent / "ssl_cert" / "server.crt"
        if key_file is None:
            key_file = Path(__file__).parent.parent / "ssl_cert" / "server.key"
        
        if use_ssl and Path(cert_file).exists() and Path(key_file).exists():
            self.ssl_context = create_ssl_context(str(cert_file), str(key_file))
            print(f"✅ SSL enabled for WebSocket server")
        elif use_ssl:
            print(f"⚠️ SSL certificates not found at {cert_file}, {key_file}")
            print("   WebSocket server will run without SSL (ws://)")
            self.use_ssl = False

    @property
    def is_connected(self) -> bool:
        return self.connected

    async def on_observation_received(self, websocket):
        try:
            async for message in websocket:
                self.last_observation = json.loads(message)
                # print("📥 Observation received:", self.last_observation)
        except websockets.ConnectionClosedOK:
            print("🔌 Connection closed normally.")
        except websockets.ConnectionClosedError as e:
            print(f"⚠️ Connection closed with error: {e}")
        except Exception as e:
            print(f"❌ Unexpected error in handler: {e}")

    async def start_websocket_server(self):
        protocol = "wss" if self.ssl_context else "ws"
        print(f"🌐 WebSocket server listening on {protocol}://0.0.0.0:8080")
        self._server = await websockets.serve(
            self.on_observation_received, 
            "0.0.0.0", 
            8080,
            ssl=self.ssl_context
        )
        self._ready_event.set()  # mark as ready once server is up
        await self._server.wait_closed()

    def _run_server_thread(self):
        """Entry point for the background thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self.start_websocket_server())
        finally:
            self._loop.close()

    def connect(self):
        """Starts the websocket server in a background thread."""
        print("🚀 Starting websocket server thread...")
        self._thread = threading.Thread(target=self._run_server_thread, daemon=True)
        self._thread.start()
        # Wait until the server is ready
        self._ready_event.wait()
        self.connected = True
        protocol = "wss" if self.ssl_context else "ws"
        print(f"✅ WebSocket server is ready and running in background ({protocol}://)")

    async def disconnect(self):
        if self._server and self._loop and self._loop.is_running():
            print("🛑 Shutting down WebSocket server...")
            # Schedule shutdown on the server loop
            def _shutdown():
                self._server.close()
            self._loop.call_soon_threadsafe(_shutdown)
        self.connected = False
