/**
 * WebSocket Manager
 * Handles connection to the VR headset server and sends controller data
 */
class WebSocketManager {
  constructor() {
    this.socket = null;
    this.isConnected = false;
    this.isToggling = false; // Prevent multiple toggle operations
    // Build WebSocket URL from current origin with port 8080
    // Use wss:// for HTTPS pages, ws:// for HTTP pages
    const origin = new URL(window.location.origin);
    const wsProtocol = origin.protocol === 'https:' ? 'wss:' : 'ws:';
    this.serverUrl = `${wsProtocol}//${origin.hostname}:8080`;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 3;
    this.resetActive = false; // Flag to send reset=true with controller updates
    
    // Store controller data
    this.controllerData = {
      left: null,
      right: null
    };
  }

  /**
   * Connect to the WebSocket server
   * @returns {Promise<boolean>} - True if connected successfully
   */
  connect() {
    return new Promise((resolve, reject) => {
      if (this.isConnected && this.socket) {
        console.log('🔌 WebSocket already connected');
        resolve(true);
        return;
      }

      console.log(`🔌 Connecting to WebSocket server at ${this.serverUrl}...`);
      
      try {
        this.socket = new WebSocket(this.serverUrl);

        this.socket.onopen = () => {
          console.log('✅ WebSocket connected successfully');
          this.isConnected = true;
          this.reconnectAttempts = 0;
          resolve(true);
        };

        this.socket.onclose = (event) => {
          console.log(`🔌 WebSocket disconnected (code: ${event.code})`);
          this.isConnected = false;
          this.socket = null;
        };

        this.socket.onerror = (error) => {
          console.error('❌ WebSocket error:', error);
          this.isConnected = false;
          reject(error);
        };

        this.socket.onmessage = (event) => {
          // Handle incoming messages if needed
          console.log('📥 Received message:', event.data);
        };

      } catch (error) {
        console.error('❌ Failed to create WebSocket:', error);
        reject(error);
      }
    });
  }

  /**
   * Disconnect from the WebSocket server
   */
  disconnect() {
    if (this.socket) {
      console.log('🔌 Disconnecting WebSocket...');
      this.socket.close();
      this.socket = null;
      this.isConnected = false;
    }
  }

  /**
   * Toggle connection state
   * @returns {Promise<boolean>} - New connection state
   */
  async toggleConnection() {
    // Prevent multiple simultaneous toggle operations
    if (this.isToggling) {
      console.log('⏳ Toggle already in progress, ignoring...');
      return this.isConnected;
    }
    
    this.isToggling = true;
    
    try {
      if (this.isConnected) {
        this.disconnect();
        return false;
      } else {
        try {
          await this.connect();
          return true;
        } catch (error) {
          return false;
        }
      }
    } finally {
      // Small delay to prevent rapid re-triggers
      setTimeout(() => {
        this.isToggling = false;
      }, 300);
    }
  }

  /**
   * Update controller pose data (matches PosePacket structure)
   * @param {string} hand - 'left' or 'right'
   * @param {number[]} pos - [x, y, z] position array
   * @param {number[]} rot - [x, y, z, w] quaternion rotation array
   * @param {number} joystickX - X-axis joystick value
   * @param {boolean} enabled - Whether the controller is enabled/grabbing
   */
  updateControllerData(hand, pos, rot, joystickX = 0, enabled = false) {
    this.controllerData[hand] = {
      pos: pos,
      rot: rot,
      joystickX: joystickX,
      enabled: enabled
    };
  }

  /**
   * Send current controller data to the server (matches FramePacket structure)
   */
  sendControllerData() {
    if (!this.isConnected || !this.socket || this.socket.readyState !== WebSocket.OPEN) {
      return;
    }

    // FramePacket structure: { left: PosePacket, right: PosePacket }
    const payload = {
      reset: this.resetActive,
      left: this.controllerData.left,
      right: this.controllerData.right
    };

    try {
      this.socket.send(JSON.stringify(payload));
    } catch (error) {
      console.error('❌ Failed to send controller data:', error);
    }
  }

  /**
   * Trigger reset mode for a specified duration
   * @param {number} durationMs - Duration in milliseconds to keep reset active
   * @returns {Promise<void>} - Resolves when reset period is complete
   */
  triggerReset(durationMs = 500) {
    return new Promise((resolve) => {
      this.resetActive = true;
      console.log('🔄 Reset triggered');
      
      setTimeout(() => {
        this.resetActive = false;
        console.log('🔄 Reset complete');
        resolve();
      }, durationMs);
    });
  }
}

// Global singleton instance
window.webSocketManager = new WebSocketManager();
