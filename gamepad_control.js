function move(axVal) {

  // If the manual/autonomus control select is in the manual position, you can publish

  if (document.getElementById('controlCheck').checked) {



    var twist = new ROSLIB.Message({

        linear : {

              /*
                When the leftJoystickVert axis is pushed up, axVal[leftJoystickVert] is output a negative value
                When the leftJoystickVert axis is pushed down, axVal[leftJoystickVert] is output a positive value
              */

            x : -(axVal[leftJoystickVert] - axisOffset[leftJoystickVert]),

            y : 0,

            z : 0

        },



        angular : {

              /*
                When the rightJoystickHorz axis is pushed left, axVal[rightJoystickHorz] is output a negative value
                When the rightJoystickHorz axis is pushed right, axVal[rightJoystickHorz] is output a positive value
              */

            x : 0,

            y : 0,

            z : axVal[rightJoystickHorz] - axisOffset[rightJoystickHorz]

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

/* Use the variables in this function to identify analog and button mapping of the controlller
 The data from the game controller axis is stored in the "gp.axis" variable
 gp.axis is an array with 4 elements; axis[0] (Left Horizontal axis), axis[1](Left Vertical axis), axis[2] (Right Horizontal axis), axis[3] (Right Veritcal axis)
*/

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

    move([0, 0, 0, 0]);

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

move([0, 0, 0, 0]);