var gpio = require('gpio');

gpio.initialize();

gpio.on('initialize', function() {
  console.log("Setting GPIO_509=OUT");
  gpio.setPin(509, "out");
  console.log("Setting GPIO_509=HIGH");
  gpio.writePin(509, 1);
});
