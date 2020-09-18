function updateChart(chart, newData, name, count) {
    // Prevent too many items from being added to the charts
    if (count < MAX_PLOT_IDX) {
      // Add the new name (the sample measured time) to the chart
      chart.data.labels.push(name);
      // Add the x, y, and z data items to the chart
      chart.data.datasets.forEach((dataset, index) => {
        dataset.data.push(newData[index]);
      });
    } else {
  
      // Add the new name (the sample measured time) to the chart
      chart.data.labels.push(name);
      chart.data.datasets.forEach((dataset, index) => {
        dataset.data.push(newData[index]);
        // remove the oldest datapoint form the chart
        dataset.data.splice(0, 1);
      });
      // remove the oldest datapoint's label form the chart
      chart.data.labels.splice(0, 1);
    }
    // Update the chart with no delay
    chart.update(0);
  }
  
  function getChartTime(unixTime){
    var date = new Date(unixTime*1000);
    var msgTime_Formatted = "" + date.getHours() + ":" + date.getMinutes() + ":" + date.getSeconds() + "." + date.getMilliseconds();
  
    // This if-else statement ensure that the milliseconds are zero-terminated if they legth is less than 3-decimal places
    // It ensures tht the milliseconds are all displayed as the same length to make the times mroe readable
    if(date.getMilliseconds() < 10){
      msgTime_Formatted = msgTime_Formatted.padEnd(msgTime_Formatted.length + 2, "0");
    }
    else if(date.getMilliseconds() < 100){
      msgTime_Formatted = msgTime_Formatted.padEnd(msgTime_Formatted.length + 1, "0");
    }
    return msgTime_Formatted;
  }
  
  function updateGamepadChart(idx, val){
  /* 
      idx=0  controls the left joystick
      idx=1 controls the right joystick
  */
    if(idx == 0){
      Chart_Controller.data.datasets[idx].data[0].y = val;
    }
    else if(idx == 1){
      Chart_Controller.data.datasets[idx].data[0].x = val;
    }
    Chart_Controller.update(0);
  }
  