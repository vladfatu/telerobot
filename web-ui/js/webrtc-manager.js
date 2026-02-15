/**
 * WebRTC Connection Manager
 * Handles WebRTC connections to camera streams from the server
 */
const WebRTCManager = {
  serverUrl: window.location.origin,
  connections: {},

  /**
   * Fetch the list of available cameras from the server
   * @returns {Promise<string[]>} Array of camera names
   */
  async getCameras() {
    try {
      const response = await fetch(`${this.serverUrl}/cameras`);
      return await response.json();
    } catch (error) {
      console.error('Failed to fetch cameras:', error);
      return [];
    }
  },

  /**
   * Connect to a camera stream via WebRTC
   * @param {string} cameraName - The name of the camera to connect to
   * @param {HTMLVideoElement} videoElement - The video element to stream to
   * @returns {Promise<boolean>} True if connection was successful
   */
  async connectToCamera(cameraName, videoElement) {
    try {
      const pc = new RTCPeerConnection({
        iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
      });

      pc.ontrack = (event) => {
        console.log(`Received track for camera: ${cameraName}`);
        videoElement.srcObject = event.streams[0];
        videoElement.play().catch(e => console.log('Autoplay prevented:', e));
      };

      pc.oniceconnectionstatechange = () => {
        console.log(`ICE state for ${cameraName}: ${pc.iceConnectionState}`);
      };

      // Create offer
      pc.addTransceiver('video', { direction: 'recvonly' });
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);

      // Send offer to server
      const response = await fetch(`${this.serverUrl}/offer`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          sdp: offer.sdp,
          type: offer.type,
          camera: cameraName
        })
      });

      const answer = await response.json();
      await pc.setRemoteDescription(new RTCSessionDescription(answer));

      this.connections[cameraName] = pc;
      console.log(`Connected to camera: ${cameraName}`);
      return true;
    } catch (error) {
      console.error(`Failed to connect to camera ${cameraName}:`, error);
      return false;
    }
  },

  /**
   * Disconnect from a camera stream
   * @param {string} cameraName - The name of the camera to disconnect from
   */
  disconnect(cameraName) {
    if (this.connections[cameraName]) {
      this.connections[cameraName].close();
      delete this.connections[cameraName];
      console.log(`Disconnected from camera: ${cameraName}`);
    }
  },

  /**
   * Disconnect from all camera streams
   */
  disconnectAll() {
    Object.keys(this.connections).forEach(cameraName => {
      this.disconnect(cameraName);
    });
  }
};

// Export for use in other modules
window.WebRTCManager = WebRTCManager;
