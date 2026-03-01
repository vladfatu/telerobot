/**
 * WebSocket Manager
 * Handles connection to the VR headset server and sends controller data
 */
class WebSocketManager {
  // Connection status constants
  static STATUS_DISCONNECTED = 'disconnected';
  static STATUS_CONNECTING = 'connecting';
  static STATUS_CONNECTED = 'connected';

  constructor() {
    this.socket = null;
    this.connectionStatus = WebSocketManager.STATUS_DISCONNECTED;
    // Build WebSocket URL from current origin with port 8080
    // Use wss:// for HTTPS pages, ws:// for HTTP pages
    const origin = new URL(window.location.origin);
    const wsProtocol = origin.protocol === 'https:' ? 'wss:' : 'ws:';
    this.serverUrl = `${wsProtocol}//${origin.hostname}:8080`;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 3;
    this.currentAction = 'none'; // Current action: none, reset, start_episode, stop_episode, save_dataset
    this.actionLocked = false; // Prevents multiple actions within 500ms
    this._connectionObservers = []; // Observers for connection state changes
    
    // Store controller data
    this.controllerData = {
      left: null,
      right: null
    };
  }

  /** @returns {boolean} Whether the connection is currently established */
  get isConnected() {
    return this.connectionStatus === WebSocketManager.STATUS_CONNECTED;
  }

  /**
   * Subscribe to connection status changes.
   * @param {function(string)} callback - Called with status string when connection state changes
   */
  addConnectionChangeListener(callback) {
    this._connectionObservers.push(callback);
  }

  /**
   * Unsubscribe from connection status changes.
   * @param {function(string)} callback - The callback to remove
   */
  removeConnectionChangeListener(callback) {
    this._connectionObservers = this._connectionObservers.filter(cb => cb !== callback);
  }

  /**
   * Update connection status and notify all observers.
   * @param {string} status - One of: 'disconnected', 'connecting', 'connected'
   */
  _setConnectionStatus(status) {
    this.connectionStatus = status;
    for (const cb of this._connectionObservers) {
      try { cb(status); } catch (e) { console.error('Observer error:', e); }
    }
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
      this._setConnectionStatus(WebSocketManager.STATUS_CONNECTING);
      
      try {
        this.socket = new WebSocket(this.serverUrl);

        this.socket.onopen = () => {
          console.log('✅ WebSocket connected successfully');
          this._setConnectionStatus(WebSocketManager.STATUS_CONNECTED);
          this.reconnectAttempts = 0;
          resolve(true);
        };

        this.socket.onclose = (event) => {
          console.log(`🔌 WebSocket disconnected (code: ${event.code})`);
          this._setConnectionStatus(WebSocketManager.STATUS_DISCONNECTED);
          this.socket = null;
        };

        this.socket.onerror = (error) => {
          console.error('❌ WebSocket error:', error);
          this._setConnectionStatus(WebSocketManager.STATUS_DISCONNECTED);
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
      this._setConnectionStatus(WebSocketManager.STATUS_DISCONNECTED);
    }
  }

  /**
   * Toggle connection state
   * @returns {Promise<boolean>} - New connection state
   */
  async toggleConnection() {
    // Prevent toggle while connecting
    if (this.connectionStatus === WebSocketManager.STATUS_CONNECTING) {
      console.log('⏳ Connection in progress, ignoring...');
      return this.isConnected;
    }
    
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
      action: this.currentAction,
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
   * Trigger an action for a specified duration
   * @param {string} action - One of: 'reset', 'start_episode', 'stop_episode', 'save_dataset'
   * @param {number} durationMs - Duration in milliseconds to keep action active
   * @returns {Promise<void>} - Resolves when action period is complete
   */
  triggerAction(action, durationMs = 500) {
    return new Promise((resolve, reject) => {
      if (this.actionLocked) {
        console.warn('⏳ Action locked, ignoring:', action);
        reject(new Error('Action locked'));
        return;
      }

      const validActions = ['reset', 'start_episode', 'stop_episode', 'save_dataset'];
      if (!validActions.includes(action)) {
        console.error('❌ Invalid action:', action);
        reject(new Error('Invalid action: ' + action));
        return;
      }

      this.actionLocked = true;
      this.currentAction = action;
      console.log(`🎬 Action triggered: ${action}`);
      
      setTimeout(() => {
        this.currentAction = 'none';
        this.actionLocked = false;
        console.log(`🎬 Action complete: ${action}`);
        resolve();
      }, durationMs);
    });
  }
}

// Global singleton instance
window.webSocketManager = new WebSocketManager();
