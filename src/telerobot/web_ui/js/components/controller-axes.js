/**
 * Controller Axes Component
 * Displays 3D axis arrows (X, Y, Z) on top of controllers with value labels
 * When grab button is held, shows delta values from the grab start position
 */
AFRAME.registerComponent('controller-axes', {
  schema: {
    arrowLength: { type: 'number', default: 0.05 },
    arrowRadius: { type: 'number', default: 0.001 },
    coneLength: { type: 'number', default: 0.02 },
    coneRadius: { type: 'number', default: 0.008 },
    offset: { type: 'vec3', default: { x: 0, y: 0, z: 0 } },
    showPosition: { type: 'boolean', default: true },
    showRotation: { type: 'boolean', default: false },
    textSize: { type: 'number', default: 0.02 }
  },

  init: function() {
    // Grab state tracking
    this.isGrabbing = false;
    this.grabStartPosition = new THREE.Vector3();
    this.grabStartRotation = new THREE.Euler();
    this.grabYawAngle = 0; // Y-axis rotation at grab time
    this.grabYawQuaternion = new THREE.Quaternion(); // For rotating axes to match grab direction
    
    // Determine which hand this controller is
    this.hand = this.el.id === 'right-hand' ? 'right' : 'left';
    
    // Joystick Y value (thumbstick forward/backward)
    this.joystickY = 0;
    
    // Current delta values for WebSocket transmission
    this.currentDelta = {
      position: new THREE.Vector3(),
      rotation: new THREE.Euler()
    };
    
    // Bind event handlers
    this.onGripDown = this.onGripDown.bind(this);
    this.onGripUp = this.onGripUp.bind(this);
    this.onThumbstickMoved = this.onThumbstickMoved.bind(this);
    
    // Listen for grip/squeeze button events
    this.el.addEventListener('gripdown', this.onGripDown);
    this.el.addEventListener('gripup', this.onGripUp);
    this.el.addEventListener('squeezestart', this.onGripDown);
    this.el.addEventListener('squeezeend', this.onGripUp);
    
    // Listen for thumbstick/joystick events
    this.el.addEventListener('thumbstickmoved', this.onThumbstickMoved);
    this.el.addEventListener('axismove', this.onThumbstickMoved);
    
    this.createAxes();
    this.tick = AFRAME.utils.throttleTick(this.tick, 16, this); // ~60fps for smooth rotation
  },

  onThumbstickMoved: function(evt) {
    // Get Y axis value from thumbstick (forward/backward)
    // evt.detail.y is typically -1 (forward) to 1 (backward)
    if (evt.detail && typeof evt.detail.y === 'number') {
      this.joystickY = evt.detail.y;
    } else if (evt.detail && evt.detail.axis && evt.detail.axis.length >= 2) {
      // axismove event format: axis[0] = x, axis[1] = y
      this.joystickY = evt.detail.axis[1];
    }
  },

  onGripDown: function() {
    if (!this.el.object3D) return;
    
    this.isGrabbing = true;
    // Save current position and rotation as the reference point
    this.grabStartPosition.copy(this.el.object3D.position);
    this.grabStartRotation.copy(this.el.object3D.rotation);
    
    // Get the controller's backward direction (positive Z in local space)
    // This makes Z axis point where the back of the controller is facing
    const backward = new THREE.Vector3(0, 0, 1);
    backward.applyQuaternion(this.el.object3D.quaternion);
    
    // Calculate the Y-axis rotation (yaw) from the backward direction projected onto XZ plane
    // This gives us the angle the controller's back is pointing around the global Y axis
    this.grabYawAngle = Math.atan2(backward.x, backward.z);
    
    // Create a quaternion for this Y rotation (to rotate axes to match grab direction)
    this.grabYawQuaternion.setFromAxisAngle(new THREE.Vector3(0, 1, 0), this.grabYawAngle);
  },

  onGripUp: function() {
    this.isGrabbing = false;
  },

  createAxes: function() {
    const data = this.data;
    const offset = data.offset;

    // Container for all axes
    this.axesContainer = document.createElement('a-entity');
    this.axesContainer.setAttribute('position', `${offset.x} ${offset.y} ${offset.z}`);
    this.el.appendChild(this.axesContainer);

    // Create X axis (Red)
    this.xAxis = this.createAxis('red', { x: 1, y: 0, z: 0 }, 'X');
    this.axesContainer.appendChild(this.xAxis.container);

    // Create Y axis (Green)
    this.yAxis = this.createAxis('green', { x: 0, y: 1, z: 0 }, 'Y');
    this.axesContainer.appendChild(this.yAxis.container);

    // Create Z axis (Blue)
    this.zAxis = this.createAxis('blue', { x: 0, y: 0, z: 1 }, 'Z');
    this.axesContainer.appendChild(this.zAxis.container);
  },

  createAxis: function(color, direction, label) {
    const data = this.data;
    const container = document.createElement('a-entity');

    // Calculate rotation to align cylinder with direction
    let rotation = { x: 0, y: 0, z: 0 };
    if (direction.x === 1) {
      rotation.z = -90;
    } else if (direction.z === 1) {
      rotation.x = 90;
    }
    // Y axis stays at default rotation

    // Arrow shaft (cylinder)
    const shaft = document.createElement('a-cylinder');
    shaft.setAttribute('color', color);
    shaft.setAttribute('radius', data.arrowRadius);
    shaft.setAttribute('height', data.arrowLength);
    shaft.setAttribute('rotation', `${rotation.x} ${rotation.y} ${rotation.z}`);
    
    // Position shaft so it starts from origin
    const shaftOffset = data.arrowLength / 2;
    shaft.setAttribute('position', `${direction.x * shaftOffset} ${direction.y * shaftOffset} ${direction.z * shaftOffset}`);
    container.appendChild(shaft);

    // Arrow head (cone)
    const cone = document.createElement('a-cone');
    cone.setAttribute('color', color);
    cone.setAttribute('radius-bottom', data.coneRadius);
    cone.setAttribute('radius-top', 0);
    cone.setAttribute('height', data.coneLength);
    cone.setAttribute('rotation', `${rotation.x} ${rotation.y} ${rotation.z}`);
    
    // Position cone at end of shaft
    const coneOffset = data.arrowLength + data.coneLength / 2;
    cone.setAttribute('position', `${direction.x * coneOffset} ${direction.y * coneOffset} ${direction.z * coneOffset}`);
    container.appendChild(cone);

    // Axis label with value
    const labelText = document.createElement('a-text');
    labelText.setAttribute('value', `${label}: 0.00`);
    labelText.setAttribute('color', color);
    labelText.setAttribute('align', 'center');
    labelText.setAttribute('width', data.textSize * 25);
    labelText.setAttribute('rotation', '-90 0 0');
    
    const labelOffset = data.arrowLength + data.coneLength + 0.015;
    labelText.setAttribute('position', `${direction.x * labelOffset} ${direction.y * labelOffset} ${direction.z * labelOffset}`);
    container.appendChild(labelText);

    return { container, shaft, cone, label: labelText, axisName: label };
  },

  tick: function() {
    if (!this.el.object3D) return;

    const position = this.el.object3D.position;
    const rotation = this.el.object3D.rotation;

    // Counter-rotate the axes container to keep arrows aligned with reference frame
    if (this.axesContainer && this.axesContainer.object3D) {
      // Get the parent's (controller's) world quaternion
      const parentWorldQuaternion = new THREE.Quaternion();
      this.el.object3D.getWorldQuaternion(parentWorldQuaternion);
      
      // Invert it to cancel out all rotations from the parent chain
      parentWorldQuaternion.invert();
      
      if (this.isGrabbing) {
        // When grabbing, rotate axes to align with grab direction
        // First cancel parent rotation, then apply grab yaw rotation
        this.axesContainer.object3D.quaternion.copy(parentWorldQuaternion);
        this.axesContainer.object3D.quaternion.multiply(this.grabYawQuaternion);
      } else {
        // When not grabbing, keep axes aligned with global axes (no rotation in world space)
        this.axesContainer.object3D.quaternion.copy(parentWorldQuaternion);
      }
    }

    // Calculate display values - either delta (when grabbing) or absolute
    let displayX, displayY, displayZ;
    let prefix = '';
    
    if (this.isGrabbing) {
      prefix = 'Δ';
      if (this.data.showPosition) {
        // Calculate delta position in world space
        const deltaWorld = new THREE.Vector3(
          position.x - this.grabStartPosition.x,
          position.y - this.grabStartPosition.y,
          position.z - this.grabStartPosition.z
        );
        
        // Rotate delta into grab-relative coordinate system
        // Use inverse of grab yaw to transform world delta into grab-local delta
        const inverseGrabYaw = this.grabYawQuaternion.clone().invert();
        deltaWorld.applyQuaternion(inverseGrabYaw);
        
        displayX = deltaWorld.x;
        displayY = deltaWorld.y;
        displayZ = deltaWorld.z;
      }
      if (this.data.showRotation) {
        // For rotation, show delta relative to grab start rotation
        displayX = THREE.MathUtils.radToDeg(rotation.x - this.grabStartRotation.x);
        displayY = THREE.MathUtils.radToDeg(rotation.y - this.grabStartRotation.y);
        displayZ = THREE.MathUtils.radToDeg(rotation.z - this.grabStartRotation.z);
      }
    } else {
      if (this.data.showPosition) {
        displayX = position.x;
        displayY = position.y;
        displayZ = position.z;
      }
      if (this.data.showRotation) {
        displayX = THREE.MathUtils.radToDeg(rotation.x);
        displayY = THREE.MathUtils.radToDeg(rotation.y);
        displayZ = THREE.MathUtils.radToDeg(rotation.z);
      }
    }

    if (this.data.showPosition) {
      this.xAxis.label.setAttribute('value', `${prefix}X: ${displayX.toFixed(2)}`);
      this.yAxis.label.setAttribute('value', `${prefix}Y: ${displayY.toFixed(2)}`);
      this.zAxis.label.setAttribute('value', `${prefix}Z: ${displayZ.toFixed(2)}`);
    }

    if (this.data.showRotation) {
      this.xAxis.label.setAttribute('value', `${prefix}X: ${displayX.toFixed(0)}°`);
      this.yAxis.label.setAttribute('value', `${prefix}Y: ${displayY.toFixed(0)}°`);
      this.zAxis.label.setAttribute('value', `${prefix}Z: ${displayZ.toFixed(0)}°`);
    }
    
    // Send controller delta data via WebSocket if connected
    if (window.webSocketManager && window.webSocketManager.isConnected) {
      // Position as [x, y, z] array (delta when grabbing, zero otherwise)
      const pos = this.isGrabbing ? 
        [displayX || 0, displayY || 0, displayZ || 0] : 
        [0, 0, 0];
      
      // Rotation as quaternion [x, y, z, w] array
      // Get current quaternion from controller
      const quaternion = this.el.object3D.quaternion;
      const rot = [quaternion.x, quaternion.y, quaternion.z, quaternion.w];
      
      // enabled = isGrabbing
      const enabled = this.isGrabbing;
      
      // Update controller data in WebSocket manager (PosePacket format)
      window.webSocketManager.updateControllerData(
        this.hand,
        pos,
        rot,
        this.joystickY,
        enabled
      );
      
      // Send data (the manager will batch both controllers' data as FramePacket)
      window.webSocketManager.sendControllerData();
    }
  },

  remove: function() {
    // Remove event listeners
    this.el.removeEventListener('gripdown', this.onGripDown);
    this.el.removeEventListener('gripup', this.onGripUp);
    this.el.removeEventListener('squeezestart', this.onGripDown);
    this.el.removeEventListener('squeezeend', this.onGripUp);
    this.el.removeEventListener('thumbstickmoved', this.onThumbstickMoved);
    this.el.removeEventListener('axismove', this.onThumbstickMoved);
    
    if (this.axesContainer && this.axesContainer.parentNode) {
      this.axesContainer.parentNode.removeChild(this.axesContainer);
    }
  }
});
