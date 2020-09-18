PWM_listener.subscribe(function(message) {
  // Update the chart with the left gamepad value
  updateGamepadChart(0, message.linear.x);
  // Round and update the table with the left gamepad value
  document.getElementById("lJoystickVal").innerHTML = Math.round(message.linear.x*1000)/1000;
// Update the chart with the right gamepad value
  updateGamepadChart(1, message.angular.z);
  // Round and update the table with the right gamepad value
  document.getElementById("rJoystickVal").innerHTML = Math.round(message.angular.z*1000)/1000;

});

BatteryListener.subscribe(function(message) {
  // Battery readings are separated by commas ... so split on the commas
  var separateBattery = message.data.split(",");
  // cell readings are separated by spaces, so split on the spaces and map to a number
  bat_6s = separateBattery[0].split(" ").map(Number);
  bat_4s = separateBattery[1].split(" ").map(Number);
  bat_3sa = separateBattery[2].split(" ").map(Number);
  bat_3sb = separateBattery[3].split(" ").map(Number);

  // Update the battery chart data
  Chart_Battery.data.datasets[0].data = [bat_6s[0], bat_4s[0], bat_3sa[0], bat_3sb[0]];
  Chart_Battery.data.datasets[1].data = [bat_6s[1], bat_4s[1], bat_3sa[1], bat_3sb[1]];
  Chart_Battery.data.datasets[2].data = [bat_6s[2], bat_4s[2], bat_3sa[2], bat_3sb[2]];
  Chart_Battery.data.datasets[3].data = [bat_6s[3], bat_4s[3], 0, 0];
  Chart_Battery.data.datasets[4].data = [bat_6s[4], 0, 0, 0];
  Chart_Battery.data.datasets[5].data = [bat_6s[5], 0, 0, 0];
  // Process the battery chart update
  Chart_Battery.update(0);
});

IMU_AbsOrientationListener.subscribe(function(message) {
  // Store the time that the message was recieved
  msgTime = ((message.header.stamp.secs) + (message.header.stamp.nsecs / Math.pow(10, 9)));

  // Update at a fixed interval to make the website more responsive
  if (msgTime - IMU_OrientationLastUpdateTime >= 1 / PLOT_UPDATE_FREQUENCY) {
    // record the time that the plot was last updated
    IMU_OrientationLastUpdateTime = msgTime;
    // extract the orientation information
    orientation_deg = [message.vector.x, message.vector.y, message.vector.z];

    // Update the orientation animation visualization
    var rotation = BoatModel.rotation;
    rotation[0] = orientation_deg[0] - 90;
    rotation[1] = orientation_deg[1];
    rotation[2] = orientation_deg[2];
    BoatModel.rotation = rotation;

    // Update the orientation IMU chart
    updateChart(Chart_OrientationDeg, orientation_deg, getChartTime(msgTime), IMU_Cnt);
    IMU_Cnt++;
  }

});

IMU_LinearAccelerationListener.subscribe(function(message) {
  // Store the time that the message was recieved
  msgTime = Math.round(((message.header.stamp.secs) + (message.header.stamp.nsecs / Math.pow(10, 9))) * 10000) / 10000;

  // Update at a fixed interval to make the website more responsive
  if (msgTime - IMU_LinearAccelerationLastUpdateTime >= 1 / PLOT_UPDATE_FREQUENCY) {
    IMU_LinearAccelerationLastUpdateTime = msgTime;
    accel = [message.vector.x, message.vector.y, message.vector.z];

    updateChart(Chart_Acceleration, accel, getChartTime(msgTime), IMU_Accel_Cnt);
    IMU_Accel_Cnt++;
  }

});

IMU_AngularVelocityListener.subscribe(function(message) {
  // Store the time that the message was recieved
  msgTime = Math.round(((message.header.stamp.secs) + (message.header.stamp.nsecs / Math.pow(10, 9))) * 10000) / 10000;

  // Update at a fixed interval to make the website more responsive
  if (msgTime - IMU_AngularVelocityLastUpdateTime >= 1 / PLOT_UPDATE_FREQUENCY) {
    IMU_AngularVelocityLastUpdateTime = msgTime;
    angVel = [message.vector.x, message.vector.y, message.vector.z];

    updateChart(Chart_AngularVelocity, angVel, getChartTime(msgTime), IMU_AngVel_Cnt);
    IMU_AngVel_Cnt++;
  }

});

IMU_MagnetometerListener.subscribe(function(message) {
  // Store the time that the message was recieved
  msgTime = Math.round(((message.header.stamp.secs) + (message.header.stamp.nsecs / Math.pow(10, 9))) * 10000) / 10000;

  // Update at a fixed interval to make the website more responsive  if (msgTime - IMU_MagnetometerLastUpdateTime >= 1 / PLOT_UPDATE_FREQUENCY) {
  if (msgTime - IMU_MagnetometerLastUpdateTime >= 1 / PLOT_UPDATE_FREQUENCY) {
    IMU_MagnetometerLastUpdateTime = msgTime;
    magVec = [message.vector.x, message.vector.y, message.vector.z];

    updateChart(Chart_Magnetometer, magVec, getChartTime(msgTime), IMU_MagVec_Cnt);
    IMU_MagVec_Cnt++;
  }

});

GPS_Coordinates.subscribe(function(message) {

  GPS_fix = message.status.status; //< Have a fix? (1/0 - true/false)
  GPS_fixquality = message.status.service; //< Fix quality (0, 1, 2 = Invalid, GPS, DGPS)
  GPS_latitude = message.latitude;
  GPS_longitude = -message.longitude;
  GPS_altitude = message.altitude;

  document.getElementById("GPS_latitude").innerHTML = GPS_latitude;
  document.getElementById("GPS_longitude").innerHTML = GPS_longitude;
  // Convert the altitude from [cm] to [ft]
  document.getElementById("GPS_altitude").innerHTML = GPS_altitude * 0.0328084;

  // If there is a valid connection (GPS_fixquality != 0), update map
  if (GPS_fix == 1 && GPS_fixquality != 0) {
    // Center the map on the most recent coordinates
    map.setCenter([GPS_longitude, GPS_latitude]);
    // remove the last coordinate marker
    marker.remove();
    // Add a new GREEN marker to the map at the coordinate position
    marker = new mapboxgl.Marker({
      color: `rgb(0, 255, 0)`
    }).setLngLat([GPS_longitude, GPS_latitude]).addTo(map);
  }
  else { // If there is not a valid GPS connection
    // store the last known location in tmp
    tmp = marker.getLngLat()
    // remove the last known location marker from the map
    marker.remove();
    // Add a new RED marker to the map at the last known location
    marker = new mapboxgl.Marker({
      color: `rgb(255, 0, 0)`
    }).setLngLat(tmp).addTo(map);

  }

  if (GPS_fix == 1) {
    document.getElementById("GPS_fix").innerHTML = "Acquired";
  } else {
    document.getElementById("GPS_fix").innerHTML = "Searching";
  }

  if (GPS_fixquality == 1) {
    document.getElementById("GPS_fixquality").innerHTML = "GPS";
  } else if (GPS_fixquality == 2) {
    document.getElementById("GPS_fixquality").innerHTML = "DGPS";
  } else {
    document.getElementById("GPS_fixquality").innerHTML = "Invalid";
  }

});

LIDAR_Scan.subscribe(function(message) {
  // Store the time that the message was recieved
  msgTime = Math.round(((message.header.stamp.secs) + (message.header.stamp.nsecs / Math.pow(10, 9))) * 10000) / 10000;

  // Update at a fixed interval to make the website more responsive
  if (msgTime - LIDAR_LastUpdateTime >= 1 / LIDAR_UPDATE_FREQUENCY) {
    LIDAR_LastUpdateTime = msgTime;

    var range_min = 100; // larger than any range that can be measured
    var range_max = 0; // smaller than any range that can be measured
    var ang = message.angle_min; // the starting angle measured
    var catesian_data = []; // The array that will hold the cartesian coordinates
    var i;
    for (i = 0; i < message.ranges.length; i++) {
      // the magnitude is infinity if the range was not measured
      if(message.ranges[i] != Infinity){
        // convert polar coordinates to cartesian
        var xc = message.ranges[i] * Math.cos(ang);
        var yc = message.ranges[i] * Math.sin(ang);
        // Only consider points that are in the front of the boat
        if(-xc > 0){
          // Add the cartesian points to the plot and rotate the points to make
          // the data easier to visualize
          catesian_data.push({ x: yc, y: -xc});
          // determine if the point is at a new maximum/minimum range
          if(message.ranges[i] < range_min){
            range_min = message.ranges[i];
          }
          else if(message.ranges[i] > range_max){
            range_max = message.ranges[i];
          }
        }
      }
      ang = ang + message.angle_increment;
    }

    // Extract and calculate time that the can was recorded
    var f_secs = Math.round(message.header.stamp.nsecs / Math.pow(10,5)); // Fractional seconds
    var myDate = new Date(message.header.stamp.secs * 1000);
    // Options for the time formatter
    var options = { hour12: false, fractionalSecondDigits: 3};

    // Update the HTML sections containing the LIDAR Info
    document.getElementById("LIDAR_range_min").innerHTML = Math.round(range_min*1000)/1000;
    document.getElementById("LIDAR_range_max").innerHTML = Math.round(range_max*1000)/1000;
    document.getElementById("LIDAR_time_increment").innerHTML = Math.round(message.time_increment*1000*1000)/1000;
    document.getElementById("LIDAR_scan_time").innerHTML = Math.round(message.scan_time*1000*1000)/1000;
    document.getElementById("LIDAR_last_measured_Time").innerHTML = (myDate.toLocaleString("en-US", options) + "." + f_secs).replace(", ", "<br>");

    // Replace the LIDAR scan chart data with the data from the most recent scan
    Chart_LIDAR.data.datasets[0].data = catesian_data;
    // Update the LIDAR scan chart with no delay
    Chart_LIDAR.update(0);
  }
});

// This function will be executed whenever the "Control Mode" switch is toggled
function updateControlMode() {
  // Always stop the motors when updading the control mode
  PWM_listener.publish(zeroMotor);
  // Variable where the control message will be stored
  var controlMsg;
  // If the control mode switch is "checked", the boat should be in manual mode
  if (document.getElementById('controlCheck').checked) {
    controlMsg = new ROSLIB.Message({
      data: 0 // Send a 0 to indicate manual mode
    });
  }
  // If the control mode switch is not "checked" the boat should be in autonomus mode
  else {
    controlMsg = new ROSLIB.Message({
      // Send the number corresponding to the selected course form the drop down menu
      data: parseInt(document.getElementById('AutonomousCourseSelection').value, 10)
    });
  }
  ControlMode.publish(controlMsg); // Publish to the topic
}

updateControlMode();