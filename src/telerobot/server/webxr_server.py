import asyncio
import json
import logging
import os
import ssl
import threading
import time
import socket
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer
from aiohttp import web, web_request
from aiohttp_cors import setup as cors_setup, ResourceOptions
import av

from telerobot import PACKAGE_DIR
from telerobot.logger import log_message

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"

class CameraStreamTrack(VideoStreamTrack):
    def __init__(self, camera_name: str):
        super().__init__()
        self.camera_name = camera_name
        self.current_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        
    def update_frame(self, frame: np.ndarray):
        with self.frame_lock:
            self.current_frame = frame.copy() if frame is not None else None
    
    async def recv(self):
        pts, time_base = await self.next_timestamp()
        
        with self.frame_lock:
            if self.current_frame is not None:
                rgb_frame = self.current_frame
            else:
                rgb_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                
        frame = av.VideoFrame.from_ndarray(rgb_frame, format="rgb24")
        frame.pts = pts
        frame.time_base = time_base
        
        return frame

class WebXRServer:
    async def health_check(self, request):
        return web.json_response({"status": "ok", "cameras": list(self.camera_tracks.keys())})

    def __init__(self, host: str = "0.0.0.0", port: int = 8765, ssl_context=None, dataset_configured: bool = False):
        self.host = host
        self.port = port
        self.ssl_context = ssl_context
        self.dataset_configured = dataset_configured
        self.app = web.Application()
        self.pcs: set = set()
        self.camera_tracks: Dict[str, CameraStreamTrack] = {}
        
        cors = cors_setup(self.app, defaults={
            "*": ResourceOptions(
                    allow_credentials=True,
                    expose_headers="*",
                    allow_headers="*",
                )
        })
        
        self.app.router.add_get("/", self.index)
        self.app.router.add_post("/offer", self.offer)
        self.app.router.add_get("/cameras", self.get_cameras)
        self.app.router.add_get("/health", self.health_check)
        self.app.router.add_get("/favicon.ico", self.favicon)
        
        web_ui_path = PACKAGE_DIR / "web_ui"
        if web_ui_path.exists():
            self.app.router.add_static("/js/", web_ui_path / "js", name="js")
        
        for route in list(self.app.router.routes()):
            cors.add(route)
            
        if os.getenv("TELEROBOT_DEBUG", "0") == "1":
            logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
    def add_camera(self, camera_name: str):
        self.camera_tracks[camera_name] = CameraStreamTrack(camera_name)
        self.logger.info(f"Added camera: {camera_name}")
    
    def update_camera_frame(self, camera_name: str, frame: np.ndarray):
        if camera_name in self.camera_tracks:
            self.camera_tracks[camera_name].update_frame(frame)
        else:
            self.logger.warning(f"Camera frame received for unknown camera: {camera_name}")
    
    async def index(self, request):
        html_path = PACKAGE_DIR / 'web_ui' / 'index.html'
        
        try:
            with open(html_path, 'r', encoding='utf-8') as f:
                html_content = f.read()
            config_script = (
                f'<script>window.telerobotConfig = '
                f'{{ "datasetConfigured": {str(self.dataset_configured).lower()} }};</script>\n'
            )
            html_content = html_content.replace('<script src="js/websocket-manager.js"></script>',
                config_script + '    <script src="js/websocket-manager.js"></script>')
            return web.Response(text=html_content, content_type='text/html')
        except FileNotFoundError:
            return web.Response(
                text="<h1>Error: web-ui/index.html not found</h1>", 
                content_type='text/html',
                status=404
            )
    
    async def favicon(self, request):
        return web.Response(status=204)

    async def get_cameras(self, request):
        return web.json_response(list(self.camera_tracks.keys()))
    
    async def wait_for_ice_gathering_complete(self, pc, timeout: float = 5.0):
        if pc.iceGatheringState == "complete":
            return

        done = asyncio.Event()

        @pc.on("icegatheringstatechange")
        async def on_icegatheringstatechange():
            if os.getenv("TELEROBOT_DEBUG", "0") == "1":
                self.logger.info(f"ICE gathering state: {pc.iceGatheringState}")
            if pc.iceGatheringState == "complete":
                done.set()

        try:
            await asyncio.wait_for(done.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            self.logger.warning("ICE gathering wait timed out; continuing with current candidates.")

    async def offer(self, request):
        params = await request.json()
        camera_name = params.get('camera', 'default')
        
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        
        pc = RTCPeerConnection()
        self.pcs.add(pc)
        
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            if pc.connectionState == "connected":
                print(f"✅ WebRTC camera connected: {camera_name}", flush=True)
            elif pc.connectionState == "failed":
                print(f"⚠️ WebRTC camera failed: {camera_name}", flush=True)
            elif os.getenv("TELEROBOT_DEBUG", "0") == "1":
                self.logger.info(f"Connection state for {camera_name}: {pc.connectionState}")
            if pc.connectionState == "failed":
                await pc.close()
                if pc in self.pcs:
                    self.pcs.discard(pc)
        
        if camera_name in self.camera_tracks:
            pc.addTrack(self.camera_tracks[camera_name])
        
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        # This app does not implement trickle ICE, so wait until candidates are in the SDP.
        await self.wait_for_ice_gathering_complete(pc)

        if os.getenv("TELEROBOT_DEBUG", "0") == "1":
            self.logger.info(f"Returning WebRTC answer for {camera_name}; ICE gathering={pc.iceGatheringState}")
        
        return web.json_response({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        })
    
    async def start_server(self):
        @web.middleware
        async def logging_middleware(request, handler):
            start_time = time.time()
            try:
                self.logger.info(f"Incoming request: {request.method} {request.path} from {request.remote}")
                response = await handler(request)
                process_time = time.time() - start_time
                self.logger.info(f"Response: {response.status} in {process_time:.3f}s")
                return response
            except Exception as e:
                self.logger.error(f"Error processing request: {e}")
                return web.Response(status=500, text=f"Server error: {str(e)}")
        
        self.app.middlewares.append(logging_middleware)
        
        runner = web.AppRunner(self.app)
        await runner.setup()
        
        if self.ssl_context:
            site = web.TCPSite(runner, self.host, self.port, ssl_context=self.ssl_context)
            protocol = "https"
        else:
            site = web.TCPSite(runner, self.host, self.port)
            protocol = "http"
            
        await site.start()
        self.logger.info(f"WebRTC Camera Server started on {protocol}://{self.host}:{self.port}")
    
    def run_in_thread(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        async def run():
            await self.start_server()
            try:
                await asyncio.Future()
            except asyncio.CancelledError:
                pass
        
        loop.run_until_complete(run())

def create_ssl_context(cert_file: str, key_file: str):
    ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ssl_context.load_cert_chain(cert_file, key_file)
    return ssl_context

def create_webxr_server(camera_names, use_https=False, cert_file=None, key_file=None, dataset_configured: bool = False) -> WebXRServer:
    ssl_context = None
    
    if use_https and cert_file and key_file:
        missing = [f for f in (cert_file, key_file) if not Path(f).exists()]
        if missing:
            raise FileNotFoundError(
                f"SSL certificate file(s) not found: {', '.join(missing)}\n"
            )
        ssl_context = create_ssl_context(cert_file, key_file)
        print(f"✅ SSL enabled with {cert_file}")
    
    server = WebXRServer(ssl_context=ssl_context, dataset_configured=dataset_configured)
    
    for camera_name in camera_names:
        server.add_camera(camera_name)
    
    return server

def setup_webxr_server(robot, logger, dataset_configured: bool = False):
    use_https = True
    cert_file = "ssl_cert/server.crt"
    key_file = "ssl_cert/server.key"

    camera_server = create_webxr_server(
        robot.cameras.keys(),
        use_https=use_https,
        cert_file=cert_file,
        key_file=key_file,
        dataset_configured=dataset_configured,
    )

    server_thread = threading.Thread(target=camera_server.run_in_thread, daemon=True)
    server_thread.start()

    local_ip = get_local_ip()

    if use_https:
        log_message(logger, "🔒 HTTPS WebXR server started")
        log_message(logger, f"🌐 Access address: https://{local_ip}:8765")
    else:
        log_message(logger, "🎥 HTTP WebXR server started")
        log_message(logger, f"🌐 Access address: http://{local_ip}:8765")

    return camera_server
