/**
 * WebSocket Panel Component
 * Creates a UI panel showing WebSocket connection status with connect/disconnect button
 */
AFRAME.registerComponent('websocket-panel', {
  schema: {
    width: { type: 'number', default: 0.5 },
    height: { type: 'number', default: 0.5 },
    position: { type: 'vec3', default: { x: -1.5, y: 1.2, z: -1.5 } }
  },

  init: function() {
    this.isResetting = false; // Prevent multiple reset triggers
    
    // Bind methods
    this.onConnectButtonAction = this.onConnectButtonAction.bind(this);
    this.onResetButtonAction = this.onResetButtonAction.bind(this);
    this.onPassthroughButtonAction = this.onPassthroughButtonAction.bind(this);
    this.onConnectionChange = this.onConnectionChange.bind(this);
    
    this.createPanel();
    
    // Subscribe to connection state changes
    if (window.webSocketManager) {
      window.webSocketManager.addConnectionChangeListener(this.onConnectionChange);
    }
  },

  createPanel: function() {
    const { width, height, position } = this.data;
    
    // Main panel container - use a-box for raycaster hit detection (draggable)
    this.panel = document.createElement('a-box');
    this.panel.setAttribute('class', 'draggable');
    this.panel.setAttribute('position', `${position.x} ${position.y} ${position.z}`);
    this.panel.setAttribute('width', width);
    this.panel.setAttribute('height', height);
    this.panel.setAttribute('depth', '0.02');
    this.panel.setAttribute('color', '#1a1a2e');
    this.panel.setAttribute('opacity', '0.9');
    this.panel.setAttribute('grabbable', 'startButtons: triggerdown; endButtons: triggerup');
    this.panel.setAttribute('draggable', '');
    this.panel.setAttribute('look-at-headset', 'smoothing: 0.05');
    
    // Title
    const title = document.createElement('a-text');
    title.setAttribute('value', 'WebSocket');
    title.setAttribute('align', 'center');
    title.setAttribute('position', `0 ${height * 0.32} 0.015`);
    title.setAttribute('width', width * 1.5);
    title.setAttribute('color', '#ffffff');
    this.panel.appendChild(title);
    
    // Status indicator (circle)
    this.statusIndicator = document.createElement('a-circle');
    this.statusIndicator.setAttribute('radius', '0.02');
    this.statusIndicator.setAttribute('position', `${-width * 0.3} ${height * 0.18} 0.015`);
    this.statusIndicator.setAttribute('color', '#ff4444'); // Red = disconnected
    this.panel.appendChild(this.statusIndicator);
    
    // Status text
    this.statusText = document.createElement('a-text');
    this.statusText.setAttribute('value', 'Disconnected');
    this.statusText.setAttribute('align', 'left');
    this.statusText.setAttribute('position', `${-width * 0.2} ${height * 0.18} 0.015`);
    this.statusText.setAttribute('width', width * 1.2);
    this.statusText.setAttribute('color', '#cccccc');
    this.panel.appendChild(this.statusText);
    
    // Connect/Disconnect button using vr-button component
    this.connectButton = document.createElement('a-entity');
    this.connectButton.setAttribute('vr-button', {
      width: width * 0.7,
      height: height * 0.14,
      color: '#4CAF50',
      hoverColor: '#66BB6A',
      pressedColor: '#2E7D32',
      text: 'Connect',
      textWidth: width * 1.5
    });
    this.connectButton.setAttribute('position', `0 ${height * 0.02} 0.015`);
    
    // Listen for button action event
    this.connectButton.addEventListener('button-action', this.onConnectButtonAction);
    
    this.panel.appendChild(this.connectButton);
    
    // Reset button using vr-button component
    this.resetButton = document.createElement('a-entity');
    this.resetButton.setAttribute('vr-button', {
      width: width * 0.7,
      height: height * 0.14,
      color: '#FF9800',
      hoverColor: '#FFB74D',
      pressedColor: '#E65100',
      text: 'Reset',
      textWidth: width * 1.5,
      disabled: true
    });
    this.resetButton.setAttribute('position', `0 ${-height * 0.16} 0.015`);
    
    // Listen for reset button action event
    this.resetButton.addEventListener('button-action', this.onResetButtonAction);
    
    this.panel.appendChild(this.resetButton);
    
    // Passthrough toggle button using vr-button component
    this.passthroughButton = document.createElement('a-entity');
    this.passthroughButton.setAttribute('vr-button', {
      width: width * 0.7,
      height: height * 0.14,
      color: '#607D8B',
      hoverColor: '#78909C',
      pressedColor: '#455A64',
      text: 'Passthrough: OFF',
      textWidth: width * 1.5
    });
    this.passthroughButton.setAttribute('position', `0 ${-height * 0.34} 0.015`);
    
    // Listen for passthrough button action event
    this.passthroughButton.addEventListener('button-action', this.onPassthroughButtonAction);
    
    this.panel.appendChild(this.passthroughButton);
    this.el.appendChild(this.panel);
  },

  onConnectButtonAction: async function(event) {
    if (!window.webSocketManager) {
      console.warn('⚠️ WebSocket manager not available');
      return;
    }
    
    try {
      const connected = await window.webSocketManager.toggleConnection();
      console.log(`🔌 WebSocket ${connected ? 'connected' : 'disconnected'}`);
    } catch (error) {
      console.error('❌ Connection error:', error);
    }
  },

  onResetButtonAction: async function(event) {
    // Prevent multiple reset triggers
    if (this.isResetting) {
      return;
    }
    
    if (!window.webSocketManager) {
      console.warn('⚠️ WebSocket manager not available');
      return;
    }
    
    if (!window.webSocketManager.isConnected) {
      console.warn('⚠️ WebSocket not connected');
      return;
    }
    
    this.isResetting = true;
    
    // Disable button during reset
    const buttonComponent = this.resetButton.components['vr-button'];
    if (buttonComponent) {
      buttonComponent.setDisabled(true);
      buttonComponent.setText('...');
    }
    
    try {
      await window.webSocketManager.triggerAction('reset', 2000);
      console.log('🔄 Reset complete');
    } catch (error) {
      console.error('❌ Reset error:', error);
    } finally {
      this.isResetting = false;
      if (buttonComponent) {
        buttonComponent.setDisabled(false);
        buttonComponent.setText('Reset');
      }
    }
  },

  onPassthroughButtonAction: function(event) {
    if (!window.passthroughManager) {
      console.warn('⚠️ Passthrough manager not available');
      return;
    }
    
    const enabled = window.passthroughManager.togglePassthrough();
    console.log(`👁️ Passthrough ${enabled ? 'enabled' : 'disabled'}`);
    
    // Update button text
    const buttonComponent = this.passthroughButton.components['vr-button'];
    if (buttonComponent) {
      buttonComponent.setText(`Passthrough: ${enabled ? 'ON' : 'OFF'}`);
      buttonComponent.setColor(enabled ? '#2196F3' : '#607D8B');
    }
  },

  onConnectionChange: function(status) {
    const isConnected = status === 'connected';
    const isConnecting = status === 'connecting';

    // Update status indicator
    const statusColor = isConnecting ? '#ffaa00' : (isConnected ? '#44ff44' : '#ff4444');
    this.statusIndicator.setAttribute('color', statusColor);
    
    // Update status text
    const statusLabel = isConnecting ? 'Connecting...' : (isConnected ? 'Connected' : 'Disconnected');
    this.statusText.setAttribute('value', statusLabel);
    
    // Update connect button
    const buttonComponent = this.connectButton.components['vr-button'];
    if (buttonComponent) {
      if (isConnecting) {
        buttonComponent.setDisabled(true);
        buttonComponent.setText('...');
      } else if (isConnected) {
        buttonComponent.setDisabled(false);
        buttonComponent.setColor('#f44336'); // Red for disconnect
        buttonComponent.setText('Disconnect');
      } else {
        buttonComponent.setDisabled(false);
        buttonComponent.setColor('#4CAF50'); // Green for connect
        buttonComponent.setText('Connect');
      }
    }
    
    // Update reset button - disable when not connected
    const resetButtonComponent = this.resetButton.components['vr-button'];
    if (resetButtonComponent && !this.isResetting) {
      resetButtonComponent.setDisabled(!isConnected);
    }
  },

  remove: function() {
    if (window.webSocketManager) {
      window.webSocketManager.removeConnectionChangeListener(this.onConnectionChange);
    }
    this.connectButton.removeEventListener('button-action', this.onConnectButtonAction);
    this.resetButton.removeEventListener('button-action', this.onResetButtonAction);
    this.passthroughButton.removeEventListener('button-action', this.onPassthroughButtonAction);
    
    if (this.panel && this.panel.parentNode) {
      this.panel.parentNode.removeChild(this.panel);
    }
  }
});
