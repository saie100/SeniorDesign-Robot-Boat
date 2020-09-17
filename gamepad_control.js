function move(axVal) {
  // If the manual/autonomus control select is in the manual position, you can publish
  if (document.getElementById('controlCheck').checked) {

    var twist = new ROSLIB.Message({
        linear : {

            x : axVal[leftJoystickVert] - axisOffset[leftJoystickVert],
            y : 0,
            z : 0
        },

        angular : {

            x : 0,
            y : 0,
            z : axVal[rightJoystickVert] - axisOffset[rightJoystickVert]
        }
    });
 
        PWM_listener.publish(twist);
  }

}

function gamepadHandler(event, connecting) {
  if (connecting) {
    var gp = event.gamepad;
    console.log(gp.id);
    if (true/*gp.id.localeCompare("Xbox 360 Wired Controller (Vendor: 046d Product: c21d)") == 0*/) {
      gpidx = gp.index;
      console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.", gp.index, gp.id, gp.buttons.length, gp.axes.length);
      axisZeroed = false;
      controlPolling = setInterval(updateControl, gpPollInt); // Poll the updateControl function
      document.getElementById("gamepad_connection").innerHTML = " - Press the \"Start\" controller button";
    } else {
      document.getElementById("gamepad_connection").innerHTML = " - Invalid Controller";
      console.log("Invalid Gamepad");
      gpidx = -1;
      updateControl(); // send 0 for both motors when controller is disconnected
      clearInterval(controlPolling);
      axisZeroed = false;
    }
  }
  // Will be executed if the controller disconnects
  else {
    gpidx = -1;
    clearInterval(controlPolling);
    axisZeroed = false;
    document.getElementById("gamepad_connection").innerHTML = " - Disconnected";
    PWM_listener.publish(zeroMotor);
  }
}

function updateControl() {
  var gp = navigator.getGamepads()[gpidx];
  if (gp != null && gpidx != -1) {
    if (gp.buttons[zeroAxisButton].pressed == true) {
      zeroAxis();
    }
    if (axisZeroed) { // The controller will only publish if the inputs have been zeroed
      move(gp.axes);
    }
  } else {
    move([0, 0, 0, 0, 0, 0]);
  }
}

function zeroAxis() {
  var gp = navigator.getGamepads()[gpidx];
  for (var i = 0; i < gp.axes.length; i++) {
    axisOffset[i] = gp.axes[i];
  }
  axisZeroed = true;
  document.getElementById("gamepad_connection").innerHTML = " - Connected";
}

window.addEventListener("gamepadconnected", function(e) {
  gamepadHandler(e, true);
}, false);
window.addEventListener("gamepaddisconnected", function(e) {
  gamepadHandler(e, false);
}, false);

// Tell the user to press a button on the controller to trigger the "gamepadconnected"
// event listener
document.getElementById("gamepad_connection").innerHTML = " - Press any controller button";

// Send the "stop command" to the motors
move([0, 0, 0, 0, 0, 0]);