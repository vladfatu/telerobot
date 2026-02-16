/**
 * Look At Headset Component
 * Makes panels always face the user's headset with smooth rotation
 */
AFRAME.registerComponent('look-at-headset', {
  schema: {
    enabled: { type: 'boolean', default: true },
    smoothing: { type: 'number', default: 0.1 } // Lower = smoother, higher = more responsive
  },

  init: function() {
    this.camera = null;
    this.targetRotation = new THREE.Euler();
    this.isGrabbed = false;

    // Listen for grab events to pause look-at while dragging
    this.el.addEventListener('grab-start', () => {
      this.isGrabbed = true;
    });

    this.el.addEventListener('grab-end', () => {
      this.isGrabbed = false;
    });
  },

  tick: function() {
    if (!this.data.enabled || this.isGrabbed) return;

    // Get camera reference
    if (!this.camera) {
      this.camera = this.el.sceneEl.camera;
      if (!this.camera) return;
    }

    // Get camera (headset) world position
    const cameraPosition = new THREE.Vector3();
    this.camera.getWorldPosition(cameraPosition);

    // Get panel world position
    const panelPosition = new THREE.Vector3();
    this.el.object3D.getWorldPosition(panelPosition);

    // Calculate direction from panel to camera (only on horizontal plane)
    const direction = new THREE.Vector3();
    direction.subVectors(cameraPosition, panelPosition);
    direction.y = 0; // Keep panel upright, only rotate on Y axis
    direction.normalize();

    // Calculate target Y rotation
    const targetY = Math.atan2(direction.x, direction.z);

    // Smoothly interpolate current rotation towards target
    const currentRotation = this.el.object3D.rotation;
    const smoothing = this.data.smoothing;

    // Interpolate Y rotation (handle wraparound)
    let deltaY = targetY - currentRotation.y;
    while (deltaY > Math.PI) deltaY -= Math.PI * 2;
    while (deltaY < -Math.PI) deltaY += Math.PI * 2;

    currentRotation.y += deltaY * smoothing;
  }
});
