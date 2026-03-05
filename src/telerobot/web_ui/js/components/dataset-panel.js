/**
 * Dataset Panel Component
 * Creates a UI panel for recording and saving dataset episodes
 * with a countdown timer before recording begins.
 */
AFRAME.registerComponent('dataset-panel', {
  schema: {
    width: { type: 'number', default: 0.5 },
    height: { type: 'number', default: 0.5 },
    position: { type: 'vec3', default: { x: 1.5, y: 1.2, z: -1.5 } }
  },

  init: function() {
    this.isCountingDown = false;
    this.isRecording = false;
    this.countdownValue = 0;
    this.countdownTimer = null;
    this.datasetConfigured = !!(window.telerobotConfig && window.telerobotConfig.datasetConfigured);

    // Bind methods
    this.onRecordButtonAction = this.onRecordButtonAction.bind(this);
    this.onSaveDatasetButtonAction = this.onSaveDatasetButtonAction.bind(this);
    this.onConnectionChange = this.onConnectionChange.bind(this);

    this.createPanel();

    // Subscribe to connection state changes
    if (window.webSocketManager) {
      window.webSocketManager.addConnectionChangeListener(this.onConnectionChange);
    }
  },

  createPanel: function() {
    const { width, height, position } = this.data;

    // Main panel container
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
    title.setAttribute('value', 'Dataset');
    title.setAttribute('align', 'center');
    title.setAttribute('position', `0 ${height * 0.32} 0.015`);
    title.setAttribute('width', width * 1.5);
    title.setAttribute('color', '#ffffff');
    this.panel.appendChild(title);

    // Status / countdown text (always visible)
    this.countdownText = document.createElement('a-text');
    this.countdownText.setAttribute('value', this.datasetConfigured ? 'not recording' : 'no dataset configured');
    this.countdownText.setAttribute('align', 'center');
    this.countdownText.setAttribute('position', `0 ${height * 0.08} 0.015`);
    this.countdownText.setAttribute('width', width * 2.5);
    this.countdownText.setAttribute('color', this.datasetConfigured ? '#FFD54F' : '#EF9A9A');
    this.panel.appendChild(this.countdownText);

    // Record / Save button
    this.recordButton = document.createElement('a-entity');
    this.recordButton.setAttribute('vr-button', {
      width: width * 0.7,
      height: height * 0.14,
      color: '#4CAF50',
      hoverColor: '#66BB6A',
      pressedColor: '#2E7D32',
      text: 'Record Episode',
      textWidth: width * 1.5,
      disabled: true
    });
    this.recordButton.setAttribute('position', `0 ${-height * 0.12} 0.015`);

    this.recordButton.addEventListener('button-action', this.onRecordButtonAction);

    this.panel.appendChild(this.recordButton);

    // Save Dataset button
    this.saveDatasetButton = document.createElement('a-entity');
    this.saveDatasetButton.setAttribute('vr-button', {
      width: width * 0.7,
      height: height * 0.14,
      color: '#2196F3',
      hoverColor: '#42A5F5',
      pressedColor: '#1565C0',
      text: 'Save Dataset',
      textWidth: width * 1.5,
      disabled: true
    });
    this.saveDatasetButton.setAttribute('position', `0 ${-height * 0.30} 0.015`);

    this.saveDatasetButton.addEventListener('button-action', this.onSaveDatasetButtonAction);

    this.panel.appendChild(this.saveDatasetButton);
    this.el.appendChild(this.panel);
  },

  onRecordButtonAction: async function() {
    if (this.isCountingDown) return;

    if (this.isRecording) {
      // Stop episode
      this.isRecording = false;

      if (window.webSocketManager && window.webSocketManager.isConnected) {
        try {
          await window.webSocketManager.triggerAction('stop_episode');
        } catch (error) {
          console.error('❌ stop_episode error:', error);
        }
      }

      console.log('💾 Episode saved');
      this.countdownText.setAttribute('value', 'not recording');

      const buttonComponent = this.recordButton.components['vr-button'];
      if (buttonComponent) {
        buttonComponent.setColor('#4CAF50');
        buttonComponent.setText('Record Episode');
      }
      return;
    }

    // Start countdown
    this.startCountdown();
  },

  startCountdown: function() {
    this.isCountingDown = true;
    this.countdownValue = 3;

    // Disable button during countdown
    const buttonComponent = this.recordButton.components['vr-button'];
    if (buttonComponent) {
      buttonComponent.setDisabled(true);
      buttonComponent.setText('Record Episode');
    }

    // Show countdown value
    this.countdownText.setAttribute('value', String(this.countdownValue));

    this.countdownTimer = setInterval(() => {
      this.countdownValue--;

      if (this.countdownValue > 0) {
        this.countdownText.setAttribute('value', String(this.countdownValue));
      } else {
        // Countdown finished
        clearInterval(this.countdownTimer);
        this.countdownTimer = null;
        this.isCountingDown = false;
        this.isRecording = true;

        // Show recording status
        this.countdownText.setAttribute('value', 'recording');

        // Trigger start_episode action
        if (window.webSocketManager && window.webSocketManager.isConnected) {
          window.webSocketManager.triggerAction('start_episode').catch((error) => {
            console.error('❌ start_episode error:', error);
          });
        }

        // Re-enable button and switch to "Save Episode"
        if (buttonComponent) {
          buttonComponent.setDisabled(false);
          buttonComponent.setColor('#f44336');
          buttonComponent.setText('Save Episode');
        }

        console.log('🔴 Recording started');
      }
    }, 1000);
  },

  onConnectionChange: function(status) {
    const isConnected = status === 'connected';
    const canUseDataset = isConnected && this.datasetConfigured;

    const recordBtnComp = this.recordButton.components['vr-button'];
    if (recordBtnComp && !this.isCountingDown) {
      recordBtnComp.setDisabled(!canUseDataset);
    }

    const saveBtnComp = this.saveDatasetButton.components['vr-button'];
    if (saveBtnComp) {
      saveBtnComp.setDisabled(!canUseDataset);
    }
  },

  onSaveDatasetButtonAction: async function() {
    if (!window.webSocketManager || !window.webSocketManager.isConnected) {
      console.warn('⚠️ WebSocket not connected');
      return;
    }

    const buttonComponent = this.saveDatasetButton.components['vr-button'];
    if (buttonComponent) {
      buttonComponent.setDisabled(true);
      buttonComponent.setText('...');
    }

    try {
      await window.webSocketManager.triggerAction('save_dataset');
      console.log('💾 Dataset saved');
    } catch (error) {
      console.error('❌ save_dataset error:', error);
    } finally {
      if (buttonComponent) {
        buttonComponent.setDisabled(false);
        buttonComponent.setText('Save Dataset');
      }
    }
  },

  remove: function() {
    if (this.countdownTimer) {
      clearInterval(this.countdownTimer);
    }

    if (window.webSocketManager) {
      window.webSocketManager.removeConnectionChangeListener(this.onConnectionChange);
    }
    this.recordButton.removeEventListener('button-action', this.onRecordButtonAction);
    this.saveDatasetButton.removeEventListener('button-action', this.onSaveDatasetButtonAction);

    if (this.panel && this.panel.parentNode) {
      this.panel.parentNode.removeChild(this.panel);
    }
  }
});
