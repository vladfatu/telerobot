import asyncio
import json
import ssl
import threading
import socket
from pathlib import Path
import websockets

from telerobot import PACKAGE_DIR

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"

def create_ssl_context(cert_file: str, key_file: str):
    """Create SSL context for WSS (secure WebSocket)."""
    ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ssl_context.load_cert_chain(cert_file, key_file)
    return ssl_context

class WebSocketServer:
    name = "websocket_server"

    def __init__(self, use_ssl: bool = True, cert_file: str = None, key_file: str = None):
        self.connected = False
        self.last_observation = None
        self.last_runtime_settings = None
        self._server = None
        self._loop = None
        self._thread = None
        self._ready_event = threading.Event()
        
        self.use_ssl = use_ssl
        self.ssl_context = None
        
        if cert_file is None:
            cert_file = Path("ssl_cert") / "server.crt"
        if key_file is None:
            key_file = Path("ssl_cert") / "server.key"
        
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
                data = json.loads(message)

                # Runtime settings messages come from the VR/web UI.
                # They are handled separately from controller pose frames.
                if isinstance(data, dict) and data.get("type") == "runtime_settings":
                    workspace_scale = data.get("workspace_scale")
                    gripper_trigger_mode = data.get("gripper_trigger_mode")

                    if workspace_scale in ("1:1 (Normal)", "2:1 (Delicate)") and gripper_trigger_mode in (
                        "Press Trigger to Close",
                        "Press Trigger to Open",
                    ):
                        self.last_runtime_settings = {
                            "workspace_scale": workspace_scale,
                            "gripper_trigger_mode": gripper_trigger_mode,
                        }
                    else:
                        print(f"⚠️ Ignored invalid runtime settings message: {data}")

                    continue

                self.last_observation = data
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
        self._ready_event.set()
        await self._server.wait_closed()

    def _run_server_thread(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self.start_websocket_server())
        finally:
            self._loop.close()

    def connect(self):
        print("🚀 Starting websocket server thread...")
        self._thread = threading.Thread(target=self._run_server_thread, daemon=True)
        self._thread.start()
        self._ready_event.wait()
        self.connected = True
        protocol = "wss" if self.ssl_context else "ws"
        print(f"✅ WebSocket server is ready and running in background ({protocol}://)")

    async def disconnect(self):
        if self._server and self._loop and self._loop.is_running():
            print("🛑 Shutting down WebSocket server...")
            def _shutdown():
                self._server.close()
            self._loop.call_soon_threadsafe(_shutdown)
        self.connected = False

def setup_websocket_server() -> WebSocketServer:
    teleop_device = WebSocketServer()
    teleop_device.connect()
    return teleop_device