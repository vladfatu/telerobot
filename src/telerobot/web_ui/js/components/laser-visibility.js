/**
 * Laser Visibility Component
 * Shows laser ray always visible (previously only on hover)
 */
AFRAME.registerComponent('laser-visibility', {
  init: function() {
    // Show laser immediately on init
    this.showLaser();
  },

  showLaser: function() {
    this.el.setAttribute('line', 'visible', true);
    this.el.setAttribute('line', 'opacity', 0.8);
  },

  hideLaser: function() {
    this.el.setAttribute('line', 'visible', false);
    this.el.setAttribute('line', 'opacity', 0);
  }
});
