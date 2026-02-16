/**
 * VR Button Component
 * A reusable button that shades on trigger press and fires action on trigger release
 */

AFRAME.registerComponent('vr-button', {
  schema: {
    width: { type: 'number', default: 0.15 },
    height: { type: 'number', default: 0.06 },
    color: { type: 'color', default: '#4CAF50' },
    hoverColor: { type: 'color', default: '#66BB6A' },
    pressedColor: { type: 'color', default: '#2E7D32' },
    text: { type: 'string', default: 'Button' },
    textColor: { type: 'color', default: '#ffffff' },
    textWidth: { type: 'number', default: 0.3 },
    disabled: { type: 'boolean', default: false }
  },

  init: function() {
    this.isHovered = false;
    this.isPressed = false;
    this.baseColor = this.data.color;
    
    // Bind methods
    this.onTriggerChanged = this.onTriggerChanged.bind(this);
    this.onMouseEnter = this.onMouseEnter.bind(this);
    this.onMouseLeave = this.onMouseLeave.bind(this);
    
    this.createButton();
    this.addEventListeners();
  },

  createButton: function() {
    const { width, height, color, text, textColor, textWidth } = this.data;
    
    // Button background
    this.el.setAttribute('geometry', {
      primitive: 'plane',
      width: width,
      height: height
    });
    this.el.setAttribute('material', {
      color: color,
      opacity: 0.9,
      shader: 'flat'
    });
    this.el.classList.add('clickable');
    
    // Button text
    this.buttonText = document.createElement('a-text');
    this.buttonText.setAttribute('value', text);
    this.buttonText.setAttribute('align', 'center');
    this.buttonText.setAttribute('position', '0 0 0.005');
    this.buttonText.setAttribute('width', textWidth);
    this.buttonText.setAttribute('color', textColor);
    this.el.appendChild(this.buttonText);
  },

  addEventListeners: function() {
    // Raycaster hover events
    this.el.addEventListener('raycaster-intersected', this.onMouseEnter);
    this.el.addEventListener('raycaster-intersected-cleared', this.onMouseLeave);
    
  },

  onMouseEnter: function(event) {
    if (this.data.disabled) return;

    console.log('Hover start on button:', this.data.text);
    
    this.isHovered = true;
    this.hoveringRaycaster = event.detail.el;
    
    // Listen for trigger value changes on the controller
    if (this.hoveringRaycaster) {
      this.hoveringRaycaster.addEventListener('triggerchanged', this.onTriggerChanged);
    }
    
    if (!this.isPressed) {
      this.el.setAttribute('material', 'color', this.data.hoverColor);
      this.el.setAttribute('scale', '1.05 1.05 1.05');
    }
  },

  onMouseLeave: function(event) {
    if (this.data.disabled) return;
    this.isHovered = false;
    
    // Remove trigger listener from the controller
    if (this.hoveringRaycaster) {
      this.hoveringRaycaster.removeEventListener('triggerchanged', this.onTriggerChanged);
      this.hoveringRaycaster = null;
    }
    
    // Reset if we leave while pressed (cancel the action)
    if (this.isPressed) {
      this.isPressed = false;
    }
    
    this.el.setAttribute('material', 'color', this.baseColor);
    this.el.setAttribute('scale', '1 1 1');
  },

  onTriggerChanged: function(event) {
    if (this.data.disabled || !this.isHovered) return;
    
    const triggerValue = event.detail.value;
    
    // Trigger pressed past 50% - button is pressed
    if (triggerValue > 0.5 && !this.isPressed) {
      this.isPressed = true;
      
      // Visual feedback - darken the button
      this.el.setAttribute('material', 'color', this.data.pressedColor);
      this.el.setAttribute('scale', '0.95 0.95 0.95');
    }
    // Trigger fully released (0%) while button was pressed - fire action
    else if (triggerValue === 0 && this.isPressed) {
      this.isPressed = false;
      
      // Fire the action event
      this.el.emit('button-action', { button: this.el }, false);
      
      // Return to hover state
      this.el.setAttribute('material', 'color', this.data.hoverColor);
      this.el.setAttribute('scale', '1.05 1.05 1.05');
    }
  },

  update: function(oldData) {
    // Update text if changed
    if (oldData.text !== this.data.text && this.buttonText) {
      this.buttonText.setAttribute('value', this.data.text);
    }
    
    // Update base color if changed
    if (oldData.color !== this.data.color) {
      this.baseColor = this.data.color;
      if (!this.isHovered && !this.isPressed) {
        this.el.setAttribute('material', 'color', this.baseColor);
      }
    }
    
    // Update text color if changed
    if (oldData.textColor !== this.data.textColor && this.buttonText) {
      this.buttonText.setAttribute('color', this.data.textColor);
    }
    
    // Handle disabled state
    if (this.data.disabled) {
      this.el.setAttribute('material', 'color', '#888888');
      this.el.setAttribute('scale', '1 1 1');
    }
  },

  // Public methods to update button state
  setText: function(text) {
    this.el.setAttribute('vr-button', 'text', text);
  },

  setColor: function(color) {
    this.el.setAttribute('vr-button', 'color', color);
  },

  setDisabled: function(disabled) {
    this.el.setAttribute('vr-button', 'disabled', disabled);
  },

  remove: function() {
    // Clean up event listeners
    this.el.removeEventListener('raycaster-intersected', this.onMouseEnter);
    this.el.removeEventListener('raycaster-intersected-cleared', this.onMouseLeave);
    
    if (this.hoveringRaycaster) {
      this.hoveringRaycaster.removeEventListener('triggerchanged', this.onTriggerChanged);
    }
    
    if (this.buttonText && this.buttonText.parentNode) {
      this.buttonText.parentNode.removeChild(this.buttonText);
    }
  }
});
