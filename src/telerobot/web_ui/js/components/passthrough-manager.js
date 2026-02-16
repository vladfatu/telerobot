/**
 * Passthrough Manager Component
 * Handles toggling passthrough mode on/off in AR sessions
 */
AFRAME.registerComponent('passthrough-manager', {
  init: function () {
    this.sky = document.querySelector('a-sky');
    this.floor = document.querySelector('a-plane');
    this.passthroughEnabled = false; // Start with passthrough OFF
    
    // Make this component globally accessible for the toggle button
    window.passthroughManager = this;
    
    // Listen for the session start
    this.el.sceneEl.addEventListener('enter-vr', () => {
      // Keep passthrough state as set by user
      this.updateVisibility();
    });

    this.el.sceneEl.addEventListener('exit-vr', () => {
      this.setPassthrough(false);
    });
  },
  setPassthrough: function (enabled) {
    this.passthroughEnabled = enabled;
    this.updateVisibility();
  },
  togglePassthrough: function () {
    this.passthroughEnabled = !this.passthroughEnabled;
    this.updateVisibility();
    return this.passthroughEnabled;
  },
  updateVisibility: function () {
    // Hide environment when passthrough is enabled
    this.sky.setAttribute('visible', !this.passthroughEnabled);
    if (this.floor) this.floor.setAttribute('visible', !this.passthroughEnabled);
  },
  isPassthroughEnabled: function () {
    return this.passthroughEnabled;
  }
});
