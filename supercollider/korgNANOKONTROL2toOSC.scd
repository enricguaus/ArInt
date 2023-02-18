// Title:   korgNANOKONTROL2toOSC.scd
// Author:  Enric Guaus
// Web:     enricguaus.wordpress.com
// Date:    15/05/2020
// License: This code is open and free. Feel free to use, modify and redistribute.
// Contact: Please, contact me to suggest improvements, notify errors, share aknowledgements, etc.
// Actions: Select IP and port for sending OSC messages.
//          Select "nanoKONTROL2" in MIDIIn.connect to open MIDI device.
// Description: Convert MIDI messages from KORG-nanoKONTROL2 conrtoller to osc messages.
//              Fader numbering starts at 1.
//              Fader velocity values are normalized [0..1].
//              Knob values are normalized [0..1].
//              Solo, Mute and Rec values are {0,1}.
//              Marker values (set,last, next) are {0,1}.
//              Rew, Fwd, Stop, Play, Rec are {0,1}
//              Track values (left, right) are {0,1}.
//              Cycle values are {0,1}.

(
// ------------------------------------------
// ---- PREPARE MIDI and OSC CONNECTIONS ----
// ------------------------------------------
MIDIClient.init;
MIDIClient.sources.postln;

//(
// <---- Run from here if there is another midi controller working
MIDIIn.connect(0,2);                        // <---- Select "nanoKONTROL2" position in the list (starting from 0)
~korgToPort = NetAddr("127.0.0.1", 57120); // <---- Put your IP address and port here to send OSC messages.

MIDIdef.cc(\nanoKONTROL2_fader,{|vel, note, chan, port|
	var label, value;
	//postf("vel:%, note:%,chan:%, port:%\n",vel,note,chan,port);
	postf("Sending OSC: ");
	label=postf("/nanoKONTROL2/fader/%",chan+1);
	value=vel/127;
	postf(" %\n",vel/127);
	~korgToPort.sendMsg(label,value.asString)
},7,srcID:-1901292095);

MIDIdef.cc(\nanoKONTROL2_knob,{|vel, note, chan, port|
	var label, value;
	//postf("vel:%, note:%,chan:%, port:%\n",vel,note,chan,port);
	postf("Sending OSC: ");
	label=postf("/nanoKONTROL2/knob/%",chan+1);
	value=vel/127;
	postf(" %\n",vel/127);
	~korgToPort.sendMsg(label,value.asString)
},(16..23),srcID:-1901292095);

MIDIdef.cc(\nanoKONTROL2_solo,{|vel, note, chan, port|
	var label, value;
	//postf("vel:%, note:%,chan:%, port:%\n",vel,note,chan,port);
	postf("Sending OSC: ");
	label=postf("/nanoKONTROL2/solo/%",chan+1);
	value=vel/127;
	postf(" %\n",vel/127);
	~korgToPort.sendMsg(label,value.asString)
},(32..39),srcID:-1901292095);

MIDIdef.cc(\nanoKONTROL2_mute,{|vel, note, chan, port|
	var label, value;
	//postf("vel:%, note:%,chan:%, port:%\n",vel,note,chan,port);
	postf("Sending OSC: ");
	label=postf("/nanoKONTROL2/mute/%",chan+1);
	value=vel/127;
	postf(" %\n",vel/127);
	~korgToPort.sendMsg(label,value.asString)
},(48..55),srcID:-1901292095);

MIDIdef.cc(\nanoKONTROL2_rec,{|vel, note, chan, port|
	var label, value;
	//postf("vel:%, note:%,chan:%, port:%\n",vel,note,chan,port);
	postf("Sending OSC: ");
	label=postf("/nanoKONTROL2/rec/%",chan+1);
	value=vel/127;
	postf(" %\n",vel/127);
	~korgToPort.sendMsg(label,value.asString)
},(64..71),srcID:-1901292095);

MIDIdef.cc(\nanoKONTROL2_marker,{|vel, note, chan, port|
	var label, action, value;
	//postf("vel:%, note:%,chan:%, port:%\n",vel,note,chan,port);
	postf("Sending OSC: ");
	switch(note,
		60,{action="set"},61,{action="last"},62,{action="next"}
	);
	label=postf("/nanoKONTROL2/marker/%",action);
	value=vel/127;
	postf(" %\n",vel/127);
	~korgToPort.sendMsg(label,value.asString)
},(60..62),srcID:-1901292095);

MIDIdef.cc(\nanoKONTROL2_transport,{|vel, note, chan, port|
	var label, action, value;
	postf("Sending OSC: ");
	switch(note,
		41,{action="play"}, 42,{action="stop"}, 43,{action="rew"}, 44,{action="fwd"}, 45,{action="rec"}
	);
	label=postf("/nanoKONTROL2/transport/%",action);
	value=vel/127;
	postf(" %\n",vel/127);
	~korgToPort.sendMsg(label,value.asString)
},(41..45),srcID:-1901292095);

MIDIdef.cc(\nanoKONTROL2_track,{|vel, note, chan, port|
	var label, action, value;
	postf("Sending OSC: ");
	switch(note,
		58,{action="left"}, 59,{action="right"}
	);
	label=postf("/nanoKONTROL2/track/%",action);
	value=vel/127;
	postf(" %\n",vel/127);
	~korgToPort.sendMsg(label,value.asString)
},(58..59),srcID:-1901292095);

MIDIdef.cc(\nanoKONTROL2_cycle,{|vel, note, chan, port|
	var label, value;
	//postf("vel:%, note:%,chan:%, port:%\n",vel,note,chan,port);
	postf("Sending OSC: ");
	label=postf("/nanoKONTROL2/cycle");
	value=vel/127;
	postf(" %\n",vel/127);
	~korgToPort.sendMsg(label,value.asString);
},(46),srcID:-1901292095);
)


(
// ---------------
// ---- CLOSE ----
// ---------------
// ---- OSC ----
~korgToPort.disconnect;
// ---- MIDI ----
MIDIdef.freeAll; // stop responding
MIDIIn.disconnect;
)
