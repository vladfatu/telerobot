/**
 * WebRTC Connection Manager
 * Handles WebRTC connections to camera streams from the server
 */
const WebRTCManager = {
  serverUrl: window.location.origin,
  connections: {},

  async waitForIceGatheringComplete(pc, timeoutMs = 5000) {
    if (pc.iceGatheringState === 'complete') {
      return;
    }

    console.log(`Waiting for ICE gathering. Current state: ${pc.iceGatheringState}`);

    await new Promise((resolve) => {
      const timeout = setTimeout(() => {
        console.warn('ICE gathering wait timed out; continuing with current candidates.');
        pc.removeEventListener('icegatheringstatechange', checkState);
        resolve();
      }, timeoutMs);

      function checkState() {
        console.log(`ICE gathering state: ${pc.iceGatheringState}`);
        if (pc.iceGatheringState === 'complete') {
          clearTimeout(timeout);
          pc.removeEventListener('icegatheringstatechange', checkState);
          resolve();
        }
      }

      pc.addEventListener('icegatheringstatechange', checkState);
      checkState();
    });
  },

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

      let firefoxIceWarningShown = false;
      pc.oniceconnectionstatechange = () => {
        console.log(`ICE state for ${cameraName}: ${pc.iceConnectionState}`);

        if (pc.iceConnectionState === 'failed' && !firefoxIceWarningShown) {
          firefoxIceWarningShown = true;
          console.error(
            'Camera WebRTC connection failed. If using Firefox, open about:config and set ' +
            'media.peerconnection.ice.obfuscate_host_addresses to false, then reload. ' +
            'Alternatively try Microsoft Edge or Chrome.'
          );
        }
      };

      // Create offer
      pc.addTransceiver('video', { direction: 'recvonly' });
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);

      // This app does not implement trickle ICE, so wait until candidates are in the SDP.
      await this.waitForIceGatheringComplete(pc);

      // Send offer to server
      const response = await fetch(`${this.serverUrl}/offer`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          sdp: pc.localDescription.sdp,
          type: pc.localDescription.type,
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
