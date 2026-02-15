import asyncio
import json
import logging
import ssl
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer
from aiohttp import web, web_request
from aiohttp_cors import setup as cors_setup, ResourceOptions
import av


class CameraStreamTrack(VideoStreamTrack):
    """Custom video track that streams camera frames."""
    
    def __init__(self, camera_name: str):
        super().__init__()
        self.camera_name = camera_name
        self.current_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        
    def update_frame(self, frame: np.ndarray):
        """Update the current frame to be streamed."""
        with self.frame_lock:
            self.current_frame = frame.copy() if frame is not None else None
    
    async def recv(self):
        """Generate video frames for WebRTC."""
        pts, time_base = await self.next_timestamp()
        
        with self.frame_lock:
            if self.current_frame is not None:
                # Convert BGR to RGB (OpenCV uses BGR by default)
                rgb_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
            else:
                # Create a black frame if no frame is available
                rgb_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                
        # Convert numpy array to av.VideoFrame
        frame = av.VideoFrame.from_ndarray(rgb_frame, format="rgb24")
        frame.pts = pts
        frame.time_base = time_base
        
        return frame


class WebRTCCameraServer:
    """WebRTC server for streaming multiple camera feeds."""
    
    async def health_check(self, request):
        """Simple health check endpoint."""
        return web.json_response({"status": "ok", "cameras": list(self.camera_tracks.keys())})

    def __init__(self, host: str = "0.0.0.0", port: int = 8765, ssl_context=None):
        self.host = host
        self.port = port
        self.ssl_context = ssl_context
        self.app = web.Application()
        self.pcs: set = set()
        self.camera_tracks: Dict[str, CameraStreamTrack] = {}
        
        # Setup CORS
        cors = cors_setup(self.app, defaults={
            "*": ResourceOptions(
                    allow_credentials=True,
                    expose_headers="*",
                    allow_headers="*",
                )
        })
        
        # Setup routes
        self.app.router.add_get("/", self.index)
        self.app.router.add_post("/offer", self.offer)
        self.app.router.add_get("/cameras", self.get_cameras)
        self.app.router.add_get("/health", self.health_check)
        
        # Serve static files from web-ui folder
        script_dir = Path(__file__).parent.parent  # Go up to project root
        web_ui_path = script_dir / "web-ui"
        if web_ui_path.exists():
            self.app.router.add_static("/js/", web_ui_path / "js", name="js")
        
        # Add CORS to all routes
        for route in list(self.app.router.routes()):
            cors.add(route)
            
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
    def add_camera(self, camera_name: str):
        """Add a camera stream."""
        self.camera_tracks[camera_name] = CameraStreamTrack(camera_name)
        self.logger.info(f"Added camera: {camera_name}")
    
    def update_camera_frame(self, camera_name: str, frame: np.ndarray):
        """Update frame for a specific camera."""
        if camera_name in self.camera_tracks:
            self.camera_tracks[camera_name].update_frame(frame)
    
    async def index(self, request):
        """Serve the main HTML page from web-ui folder."""
        import os
        
        # Get the project root directory (parent of server folder)
        script_dir = Path(__file__).parent.parent
        html_path = script_dir / 'web-ui' / 'index.html'
        
        try:
            with open(html_path, 'r', encoding='utf-8') as f:
                html_content = f.read()
            return web.Response(text=html_content, content_type='text/html')
        except FileNotFoundError:
            return web.Response(
                text="<h1>Error: web-ui/index.html not found</h1>", 
                content_type='text/html',
                status=404
            )
    
    async def get_cameras(self, request):
        """Return list of available cameras."""
        return web.json_response(list(self.camera_tracks.keys()))
    
    async def offer(self, request):
        """Handle WebRTC offer."""
        params = await request.json()
        camera_name = params.get('camera', 'default')
        
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        
        pc = RTCPeerConnection()
        self.pcs.add(pc)
        
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            self.logger.info(f"Connection state for {camera_name}: {pc.connectionState}")
            if pc.connectionState == "failed":
                await pc.close()
                if pc in self.pcs:
                    self.pcs.discard(pc)
        
        # Add video track
        if camera_name in self.camera_tracks:
            pc.addTrack(self.camera_tracks[camera_name])
        
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        
        return web.json_response({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        })
    
    async def start_server(self):
        """Start the WebRTC server with optional HTTPS support."""
        
        # Add middleware for better request logging
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
        
        # Add the middleware to the app
        self.app.middlewares.append(logging_middleware)
        
        runner = web.AppRunner(self.app)
        await runner.setup()
        
        # Create site with or without SSL
        if self.ssl_context:
            site = web.TCPSite(runner, self.host, self.port, ssl_context=self.ssl_context)
            protocol = "https"
        else:
            site = web.TCPSite(runner, self.host, self.port)
            protocol = "http"
            
        await site.start()
        self.logger.info(f"WebRTC Camera Server started on {protocol}://{self.host}:{self.port}")
        
        # Print network info
        import socket
        hostname = socket.gethostname()
        try:
            local_ip = socket.gethostbyname(hostname)
            self.logger.info(f"Server accessible at: {protocol}://{local_ip}:{self.port}")
        except:
            self.logger.info(f"Could not determine local IP")
    
    def run_in_thread(self):
        """Run the server in a background thread."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        async def run():
            await self.start_server()
            try:
                await asyncio.Future()  # Run forever
            except asyncio.CancelledError:
                pass
        
        loop.run_until_complete(run())


def create_ssl_context(cert_file: str, key_file: str):
    """Create SSL context for HTTPS."""
    ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ssl_context.load_cert_chain(cert_file, key_file)
    return ssl_context


def create_camera_server(camera_names, use_https=False, cert_file=None, key_file=None) -> WebRTCCameraServer:
    """Create and configure the camera server."""
    ssl_context = None
    
    if use_https and cert_file and key_file:
        if Path(cert_file).exists() and Path(key_file).exists():
            ssl_context = create_ssl_context(cert_file, key_file)
            print(f"✅ SSL enabled with {cert_file}")
        else:
            print(f"❌ SSL certificate files not found: {cert_file}, {key_file}")
            print("Falling back to HTTP")
    
    server = WebRTCCameraServer(ssl_context=ssl_context)
    
    # Add your camera streams
    for camera_name in camera_names:
        server.add_camera(camera_name)
    
    return server


if __name__ == "__main__":
    import sys
    
    # Check for HTTPS flag
    use_https = "--https" in sys.argv or "-s" in sys.argv
    cert_file = "ssl_cert/server.crt"
    key_file = "ssl_cert/server.key"
    
    server = create_camera_server(["test"], use_https=use_https, cert_file=cert_file, key_file=key_file)
    asyncio.run(server.start_server())
    print("Server running. Press Ctrl+C to stop.")
    try:
        asyncio.run(asyncio.Future())  # Run forever
    except KeyboardInterrupt:
        print("Server stopped.")
