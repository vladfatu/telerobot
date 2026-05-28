/**
 * WebSocket Panel Component
 * Creates a UI panel showing WebSocket connection status with connect/disconnect button
 */
AFRAME.registerComponent('websocket-panel', {
  schema: {
    width: { type: 'number', default: 0.5 },
    height: { type: 'number', default: 0.95 },
    position: { type: 'vec3', default: { x: -1.5, y: 1.2, z: -1.5 } }
  },

  init: function() {
    this.isResetting = false; // Prevent multiple reset triggers

    // Runtime control settings shown in VR.
    this.workspaceScale = '1:1 (Normal)';
    this.gripperTriggerMode = 'Press Trigger to Close';
    
    // Bind methods
    this.onConnectButtonAction = this.onConnectButtonAction.bind(this);
    this.onScaleButtonAction = this.onScaleButtonAction.bind(this);
    this.onTriggerModeButtonAction = this.onTriggerModeButtonAction.bind(this);
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
    title.setAttribute('position', `0 ${height * 0.39} 0.015`);
    title.setAttribute('width', width * 1.5);
    title.setAttribute('color', '#ffffff');
    this.panel.appendChild(title);
    
    // Status indicator (circle)
    this.statusIndicator = document.createElement('a-circle');
    this.statusIndicator.setAttribute('radius', '0.02');
    this.statusIndicator.setAttribute('position', `${-width * 0.3} ${height * 0.27} 0.015`);
    this.statusIndicator.setAttribute('color', '#ff4444'); // Red = disconnected
    this.panel.appendChild(this.statusIndicator);
    
    // Status text
    this.statusText = document.createElement('a-text');
    this.statusText.setAttribute('value', 'Disconnected');
    this.statusText.setAttribute('align', 'left');
    this.statusText.setAttribute('position', `${-width * 0.2} ${height * 0.27} 0.015`);
    this.statusText.setAttribute('width', width * 1.2);
    this.statusText.setAttribute('color', '#cccccc');
    this.panel.appendChild(this.statusText);
    
    // Connect/Disconnect button using vr-button component
    this.connectButton = document.createElement('a-entity');
    this.connectButton.setAttribute('vr-button', {
      width: width * 0.7,
      height: height * 0.09,
      color: '#4CAF50',
      hoverColor: '#66BB6A',
      pressedColor: '#2E7D32',
      text: 'Connect',
      textWidth: width * 1.5
    });
    this.connectButton.setAttribute('position', `0 ${height * 0.15} 0.015`);
    
    // Listen for button action event
    this.connectButton.addEventListener('button-action', this.onConnectButtonAction);
    
    this.panel.appendChild(this.connectButton);

    // Workspace scale toggle button
    this.scaleButton = document.createElement('a-entity');
    this.scaleButton.setAttribute('vr-button', {
      width: width * 0.7,
      height: height * 0.09,
      color: '#3F51B5',
      hoverColor: '#5C6BC0',
      pressedColor: '#283593',
      text: 'Scale: 1:1',
      textWidth: width * 1.5
    });
    this.scaleButton.setAttribute('position', `0 ${height * 0.03} 0.015`);
    this.scaleButton.addEventListener('button-action', this.onScaleButtonAction);
    this.panel.appendChild(this.scaleButton);

    // Gripper trigger mode toggle button
    this.triggerModeButton = document.createElement('a-entity');
    this.triggerModeButton.setAttribute('vr-button', {
      width: width * 0.7,
      height: height * 0.09,
      color: '#9C27B0',
      hoverColor: '#BA68C8',
      pressedColor: '#6A1B9A',
      text: 'Trigger: Close',
      textWidth: width * 1.5
    });
    this.triggerModeButton.setAttribute('position', `0 ${-height * 0.09} 0.015`);
    this.triggerModeButton.addEventListener('button-action', this.onTriggerModeButtonAction);
    this.panel.appendChild(this.triggerModeButton);
    
    // Reset button using vr-button component
    this.resetButton = document.createElement('a-entity');
    this.resetButton.setAttribute('vr-button', {
      width: width * 0.7,
      height: height * 0.09,
      color: '#FF9800',
      hoverColor: '#FFB74D',
      pressedColor: '#E65100',
      text: 'Reset: Hold Stick',
      textWidth: width * 1.5,
      disabled: true
    });
    this.resetButton.setAttribute('position', `0 ${-height * 0.21} 0.015`);
    
    // Listen for reset button action event
    this.resetButton.addEventListener('button-action', this.onResetButtonAction);
    
    this.panel.appendChild(this.resetButton);
    
    // Passthrough toggle button using vr-button component
    this.passthroughButton = document.createElement('a-entity');
    this.passthroughButton.setAttribute('vr-button', {
      width: width * 0.7,
      height: height * 0.09,
      color: '#607D8B',
      hoverColor: '#78909C',
      pressedColor: '#455A64',
      text: 'Passthrough: OFF',
      textWidth: width * 1.5
    });
    this.passthroughButton.setAttribute('position', `0 ${-height * 0.33} 0.015`);
    
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

  sendCurrentRuntimeSettings: function() {
    if (!window.webSocketManager) {
      console.warn('⚠️ WebSocket manager not available');
      return;
    }

    if (!window.webSocketManager.isConnected) {
      console.warn('⚠️ WebSocket not connected; runtime settings not sent');
      return;
    }

    window.webSocketManager.sendRuntimeSettings(
      this.workspaceScale,
      this.gripperTriggerMode
    );
  },

  onScaleButtonAction: function(event) {
    this.workspaceScale = this.workspaceScale === '1:1 (Normal)'
      ? '2:1 (Delicate)'
      : '1:1 (Normal)';

    const buttonComponent = this.scaleButton.components['vr-button'];
    if (buttonComponent) {
      const isDelicate = this.workspaceScale === '2:1 (Delicate)';
      buttonComponent.setText(isDelicate ? 'Scale: 2:1' : 'Scale: 1:1');
      buttonComponent.setColor(isDelicate ? '#2196F3' : '#3F51B5');
    }

    this.sendCurrentRuntimeSettings();
  },

  onTriggerModeButtonAction: function(event) {
    this.gripperTriggerMode = this.gripperTriggerMode === 'Press Trigger to Close'
      ? 'Press Trigger to Open'
      : 'Press Trigger to Close';

    const buttonComponent = this.triggerModeButton.components['vr-button'];
    if (buttonComponent) {
      const isOpen = this.gripperTriggerMode === 'Press Trigger to Open';
      buttonComponent.setText(isOpen ? 'Trigger: Open' : 'Trigger: Close');
      buttonComponent.setColor(isOpen ? '#E91E63' : '#9C27B0');
    }

    this.sendCurrentRuntimeSettings();
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
    
    // Reset is intentionally not clickable from the virtual UI anymore.
    // Trigger is used for gripper control, so reset is handled by right
    // thumbstick click-hold instead.
    const resetButtonComponent = this.resetButton.components['vr-button'];
    if (resetButtonComponent && !this.isResetting) {
      resetButtonComponent.setDisabled(true);
      resetButtonComponent.setText('Reset: Hold Stick');
    }
  },

  remove: function() {
    if (window.webSocketManager) {
      window.webSocketManager.removeConnectionChangeListener(this.onConnectionChange);
    }
    this.connectButton.removeEventListener('button-action', this.onConnectButtonAction);
    if (this.scaleButton) {
      this.scaleButton.removeEventListener('button-action', this.onScaleButtonAction);
    }
    if (this.triggerModeButton) {
      this.triggerModeButton.removeEventListener('button-action', this.onTriggerModeButtonAction);
    }
    // Reset virtual button has no action listener; reset uses thumbstick hold.
    this.passthroughButton.removeEventListener('button-action', this.onPassthroughButtonAction);
    
    if (this.panel && this.panel.parentNode) {
      this.panel.parentNode.removeChild(this.panel);
    }
  }
});
