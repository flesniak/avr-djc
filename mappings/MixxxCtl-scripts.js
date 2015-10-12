midiIds = {
  "[Channel1].rate_led" : { status : 0x90, control : 0x01 },
  "[Channel2].rate_led" : { status : 0x90, control : 0x0A },
  "[Channel1].vu_meter" : { status : 0xB0, control : 0x02 },
  "[Channel2].vu_meter" : { status : 0xB0, control : 0x03 }
};

jogInvert = 1;

// Create Controller Object
MixxxCtl = new Controller();
MixxxCtl.init = function()
{
  print("MixxxCtl init");
  MixxxCtl.reset();
  MixxxCtl.connectControls();
}

MixxxCtl.shutdown = function()
{
  MixxxCtl.reset();
}

// reset the launchpad
MixxxCtl.reset = function()
{
  // disable all LEDs
  var byteArray = [ 0xF0, 0, 0, 0, 0, 0, 0, 0xF7 ];
  midi.sendSysexMsg(byteArray,byteArray.length);
}

MixxxCtl.incomingData = function(channel, control, value, status, group)
{
  //print("inc data ch " + channel + " con " + control + " val " + value + " sta " + status + " grp " + group);
  if (control == 0x10 || control == 0x11)
    MixxxCtl.jog(group, value);
}

MixxxCtl.connectControls = function()
{
  engine.connectControl("[Channel1]", "rate", function(value) { MixxxCtl.updateRateSliderLED("[Channel1]", value); } );
  engine.connectControl("[Channel2]", "rate", function(value) { MixxxCtl.updateRateSliderLED("[Channel2]", value); } );
  engine.connectControl("[Channel1]", "VuMeter", function(value) { MixxxCtl.updateVU("[Channel1]", value); } );
  engine.connectControl("[Channel2]", "VuMeter", function(value) { MixxxCtl.updateVU("[Channel2]", value); } );
}

MixxxCtl.jog = function(channel, value)
{
  if (jogInvert)
    value = (value==0x7F)?0x01:0x7F;
  var direction = (value==0x01) ? 1 : -1;
  engine.setValue(channel, "jog", direction);
}

MixxxCtl.updateRateSliderLED = function(channel, rate)
{
  var id = midiIds[channel+".rate_led"];
  var level = 0;
  if (Math.abs(rate) == 0)
    level = 0x7F;
  midi.sendShortMsg(id.status, id.control, level);
}

MixxxCtl.updateVU = function(channel, value)
{
  var id = midiIds[channel+".vu_meter"];
  var level = Math.floor(value*0x20);
  midi.sendShortMsg(id.status, id.control, level);
}