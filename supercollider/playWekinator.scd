(
// -----------------------------------------------------------
// ---- VARIABLES & BUFFERS & GROUPS & SYNTH & OSC CONFIG ----
// -----------------------------------------------------------

// ---- Variables ----
~nDescriptors    = 44;
//~nDescriptorsSho = 18;
//~nArticulations  = 8;
~nGestures       = 4;
~lSmoothDescr    = 5;
~computeAllDescr = 0;   // TODO: Manual selection not implemented.
~desRate         = 10;// 2.5; // era 10
~artiRunning     = 0;
~rmsThreshold    = 0.002;

// ---- OSC addresses and ports ----
~korgFromAddr  = NetAddr("127.0.0.1", 57120);  // Receive actions from controller
~descToAddr    = NetAddr("127.0.0.1",  6448);  // Send descriptors to Wekinator - ANN
~artiFromPort  = 12000;                        // Receive articulations from Wekinator
~artiToAddr    = NetAddr("127.0.0.1",  6449);  // Send articulations to Wekinator - DTW
~gestFromPort  = 12001;                        // Receive gestures from Wekinator
~allToMusAddr  = NetAddr("10.8.100.218", 12345); // Send articulations and gestures to external devices
~allToVidAddr  = NetAddr("10.8.101.44", 12345); // Send articulations and gestures to external devices
//~allToMusAddr  = NetAddr("127.0.0.1", 12345); // Send articulations and gestures to external devices
//~allToVidAddr  = NetAddr("127.0.0.1", 12345); // Send articulations and gestures to external devices

// ---- Buffers ----
~micBuf= Buffer.alloc(s, 65536, bufnum:2);

// ---- Groups ----
~inGroup=Group.new(s);

// ---- Buses ----
~desBus = Bus.control(s,~nDescriptors);

// ---- Synths ----
~analIn  = Synth.new(\mic,[\bufIn, 0, \bufOut,~micBuf],~inGroup);
~analDo  = Synth.after(~analIn,\analysis,[\bufIn, ~micBuf, \rate, ~desRate ],~inGroup);

// ---------------------------------
// ---- SEND DESCRIPTORS TO WEKA----
// ---------------------------------
~desArr    = Array.fill(~nDescriptors,0.0);
~desNrmArr = Array.fill(~nDescriptors,0.0);
~desSmoArr = Array.fill(~nDescriptors,Array.fill(~lSmoothDescr,0.0));

~desTsk=Task({
	inf.do({
		var tmp,tmp2;
		~desBus.get({arg i; ~desArr=tmp.addAll(i);});
		//"original: ".post; ~desArr.postln;
		~desArr.do({arg val,idx;
			// Get max value for normalization
			if(val >= ~desNrmArr[idx],{~desNrmArr[idx]=val;});
			// Smooth values
			~desSmoArr[idx].removeAt(~desSmoArr[idx].size-1);
			tmp2=~desSmoArr[idx].addFirst(val);
			~desSmoArr[idx]=Array.newFrom(tmp2);
			~desArr[idx]=~desSmoArr[idx].mean;
		});
		//"all val.: ".post; ~desSmoArr.postln;
		//"smoothed: ".post; ~desArr.postln;
		//"RMS = ".post; ~desArr[2].post; "; Loudnes = ".post; ~desArr[5].postln;
		if(~computeAllDescr==1,{
			// Send smoothed values to Weka...
			~descToAddr.sendMsg("/wek/descriptors",*~desArr);
			// ... and normalize them for sending to external devices.
			// WARNING: Original data is lost.
			~desArr.do({arg i,idx;
				//if(~desNrmArr[idx]>0,{i=i.asFloat/~desNrmArr[idx].asFloat;});
				//~desArr[idx]=abs(i);
				if(~desNrmArr[idx]>0,{~desArr[idx]=~desArr[idx].asFloat/~desNrmArr[idx].asFloat;});
				~desArr[idx]=abs(~desArr[idx]);
				//idx.post; " ".post; ~desArr[idx].post; " ".post; ~desNrmArr[idx].postln;
			});
			~allToMusAddr.sendMsg("/descriptors/values",*~desArr);
			~allToVidAddr.sendMsg("/descriptors/values",*~desArr);
		},{
			~desShoArr=Array.with( // 18 descriptors
				~desArr[8],~desArr[9],~desArr[10],~desArr[28],~desArr[29],~desArr[30],
				~desArr[32],~desArr[33],~desArr[34],~desArr[35],~desArr[36],~desArr[37],
				~desArr[4],~desArr[5],~desArr[11],~desArr[12],~desArr[13],~desArr[14] // Trec darrers MFCC, Poso altres
				//~desArr[38],~desArr[39],~desArr[40],~desArr[41],~desArr[42],~desArr[43] // són darrers MFCC
			);
			//~desArr[2].postln;
			//~rmsThreshold.postln;

			if(or(~desArr[2]>~rmsThreshold,~artiRunning>0),{
				~descToAddr.sendMsg("/wek/descriptors",*~desShoArr);
			});
			~desArr.do({arg i,idx;
				if(~desNrmArr[idx]>0,{~desArr[idx]=~desArr[idx].asFloat/~desNrmArr[idx].asFloat;});
				~desArr[idx]=abs(~desArr[idx]);
			});
			~desShoArr=Array.with( // 18 descriptors
				~desArr[8],~desArr[9],~desArr[10],~desArr[28],~desArr[29],~desArr[30],
				~desArr[32],~desArr[33],~desArr[34],~desArr[35],~desArr[36],~desArr[37],
				~desArr[4],~desArr[5],~desArr[11],~desArr[12],~desArr[13],~desArr[14] // Trec darrers MFCC, Poso altres
				//~desArr[38],~desArr[39],~desArr[40],~desArr[41],~desArr[42],~desArr[43] // són darrers MFCC
			);
			~allToMusAddr.sendMsg("/descriptors/values",*~desShoArr);
			~allToVidAddr.sendMsg("/descriptors/values",*~desShoArr);
		});
		(1/~desRate).wait;
	});
}).play;

// ----------------------------------------
// ---- WEKA INTERACTION (DATA IN/OUT) ----
// ----------------------------------------

// ---- Send and record descriptors ----

OSCdef(\nanoKONTROL2_rec1,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/outputs",1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		~descToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/1", 1);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/1", 1);
		// Rec audio buffer
		~recBf1= Buffer.alloc(s, 65536);
		~recBf1.write("~/articulations01.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~rec001 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~recBf1],~inGroup);
	},{
		// Stop rec audio buffer
		~rec001.free; ~recBf1.close; ~recBf1.free;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/1", 0);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/1", 0);
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/stopRecording");
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/rec/1',~korgFromAddr);

OSCdef(\nanoKONTROL2_rec2,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0);
		~descToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/2", 1);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/2", 1);
		// Rec audio buffer
		~recBf2= Buffer.alloc(s, 65536);
		~recBf2.write("~/articulations02.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~rec002 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~recBf2],~inGroup);
	},{
		// Stop rec audio buffer
		~rec002.free; ~recBf2.close; ~recBf2.free;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/2", 0);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/2", 0);
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/stopRecording");
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/rec/2',~korgFromAddr);

OSCdef(\nanoKONTROL2_rec3,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0);
		~descToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/3", 1);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/3", 1);
		// Rec audio buffer
		~recBf3= Buffer.alloc(s, 65536);
		~recBf3.write("~/articulations03.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~rec003 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~recBf3],~inGroup);
	},{
		// Stop rec audio buffer
		~rec003.free; ~recBf3.close; ~recBf3.free;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/3", 0);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/3", 0);
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/stopRecording");
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/rec/3',~korgFromAddr);

OSCdef(\nanoKONTROL2_rec4,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0);
		~descToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/4", 1);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/4", 1);
		// Rec audio buffer
		~recBf4= Buffer.alloc(s, 65536);
		~recBf4.write("~/articulations04.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~rec004 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~recBf4],~inGroup);
	},{
		// Stop rec audio buffer
		~rec004.free; ~recBf4.close; ~recBf4.free;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/4", 0);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/4", 0);
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/stopRecording");
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/rec/4',~korgFromAddr);

OSCdef(\nanoKONTROL2_rec5,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0);
		~descToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/5", 1);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/5", 1);
		// Rec audio buffer
		~recBf5= Buffer.alloc(s, 65536);
		~recBf5.write("~/articulations05.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~rec005 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~recBf5],~inGroup);
	},{
		// Stop rec audio buffer
		~rec005.free; ~recBf5.close; ~recBf5.free;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/5", 0);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/5", 0);
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/stopRecording");
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/rec/5',~korgFromAddr);

OSCdef(\nanoKONTROL2_rec6,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0);
		~descToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/6", 1);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/6", 1);
		// Rec audio buffer
		~recBf6= Buffer.alloc(s, 65536);
		~recBf6.write("~/articulations06.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~rec006 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~recBf6],~inGroup);
	},{
		// Stop rec audio buffer
		~rec006.free; ~recBf6.close; ~recBf6.free;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/6", 0);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/6", 0);
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/stopRecording");
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/rec/6',~korgFromAddr);

OSCdef(\nanoKONTROL2_rec7,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0);
		~descToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/7", 1);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/7", 1);
		// Rec audio buffer
		~recBf7= Buffer.alloc(s, 65536);
		~recBf7.write("~/articulations07.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~rec007 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~recBf7],~inGroup);
	},{
		// Stop rec audio buffer
		~rec007.free; ~recBf7.close; ~recBf7.free;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/7", 0);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/7", 0);
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/stopRecording");
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/rec/7',~korgFromAddr);

OSCdef(\nanoKONTROL2_rec8,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0);
		~descToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/8", 1);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/8", 1);
		// Rec audio buffer
		~recBf8= Buffer.alloc(s, 65536);
		~recBf8.write("~/articulations08.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~rec008 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~recBf8],~inGroup);
	},{
		// Stop rec audio buffer
		~rec008.free; ~recBf8.close; ~recBf8.free;
		// External control
		~allToMusAddr.sendMsg("/articulations/ctl/rec/8", 0);
		~allToVidAddr.sendMsg("/articulations/ctl/rec/8", 0);
		// Wekinator control
		~descToAddr.sendMsg("/wekinator/control/stopRecording");
		~descToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/rec/8',~korgFromAddr);

// ---- Send and record gestures ----

OSCdef(\nanoKONTROL2_solo1,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~artiToAddr.sendMsg("/wekinator/control/outputs",1.0,0.0,0.0,0.0);
		~artiToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/rec/1", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/rec/1", 1);
		// Rec audio buffer
		~solBf1= Buffer.alloc(s, 65536);
		~solBf1.write("~/gestures01.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~sol001 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~solBf1],~inGroup);
	},{
		// Stop rec audio buffer
		~sol001.free; ~solBf1.close; ~solBf1.free;
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/rec/1", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/rec/1", 0);
		// Wekinator control
		~artiToAddr.sendMsg("/wekinator/control/stopRecording");
		~artiToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/solo/1',~korgFromAddr);

OSCdef(\nanoKONTROL2_solo2,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~artiToAddr.sendMsg("/wekinator/control/outputs",0.0,1.0,0.0,0.0);
		~artiToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/rec/2", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/rec/2", 1);
		// Rec audio buffer
		~solBf2= Buffer.alloc(s, 65536);
		~solBf2.write("~/gestures02.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~sol002 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~solBf2],~inGroup);
	},{
		// Stop rec audio buffer
		~sol002.free; ~solBf2.close; ~solBf2.free;
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/rec/2", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/rec/2", 0);
		// Wekinator control
		~artiToAddr.sendMsg("/wekinator/control/stopRecording");
		~artiToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/solo/2',~korgFromAddr);

OSCdef(\nanoKONTROL2_solo3,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~artiToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,1.0,0.0);
		~artiToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/rec/3", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/rec/3", 1);
		// Rec audio buffer
		~solBf3= Buffer.alloc(s, 65536);
		~solBf3.write("~/gestures03.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~sol003 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~solBf3],~inGroup);
	},{
		// Stop rec audio buffer
		~sol003.free; ~solBf3.close; ~solBf3.free;
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/rec/3", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/rec/3", 0);
		// Wekinator control
		~artiToAddr.sendMsg("/wekinator/control/stopRecording");
		~artiToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/solo/3',~korgFromAddr);

OSCdef(\nanoKONTROL2_solo4,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// Wekinator control
		~artiToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,1.0);
		~artiToAddr.sendMsg("/wekinator/control/startRecording");
		"Sending OSC: /wekinator/control/startRecording".postln;
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/rec/4", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/rec/4", 1);
		// Rec audio buffer
		~solBf4= Buffer.alloc(s, 65536);
		~solBf4.write("~/gestures04.aiff".standardizePath, "aiff", "int16", 0, 0, true);
		~sol004 = Synth.after(~analDo,\record2disk, [\bufIn, ~micBuf, \bufOut, ~solBf4],~inGroup);
	},{
		// Stop rec audio buffer
		~sol004.free; ~solBf4.close; ~solBf4.free;
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/rec/4", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/rec/4", 0);
		// Wekinator control
		~artiToAddr.sendMsg("/wekinator/control/stopRecording");
		~artiToAddr.sendMsg("/wekinator/control/outputs",0.0,0.0,0.0,0.0);
		"Sending OSC: /wekinator/control/stopRecording".postln;
	});
},'/nanoKONTROL2/solo/4',~korgFromAddr);

// ---- Control Picture on/off to Visuals ----

OSCdef(\nanoKONTROL2_mute1,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/1", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/show/1", 1);
	},{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/1", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/show/1", 0);
	});
},'/nanoKONTROL2/mute/1',~korgFromAddr);

OSCdef(\nanoKONTROL2_mute2,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/2", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/show/2", 1);
	},{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/2", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/show/2", 0);
	});
},'/nanoKONTROL2/mute/2',~korgFromAddr);

OSCdef(\nanoKONTROL2_mute3,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/3", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/show/3", 1);
	},{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/3", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/show/3", 0);
	});
},'/nanoKONTROL2/mute/3',~korgFromAddr);

OSCdef(\nanoKONTROL2_mute4,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/4", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/show/4", 1);
	},{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/4", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/show/4", 0);
	});
},'/nanoKONTROL2/mute/4',~korgFromAddr);

OSCdef(\nanoKONTROL2_mute5,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/5", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/show/5", 1);
	},{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/5", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/show/5", 0);
	});
},'/nanoKONTROL2/mute/5',~korgFromAddr);

OSCdef(\nanoKONTROL2_mute6,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/6", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/show/6", 1);
	},{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/6", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/show/6", 0);
	});
},'/nanoKONTROL2/mute/6',~korgFromAddr);

OSCdef(\nanoKONTROL2_mute7,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/7", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/show/7", 1);
	},{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/7", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/show/7", 0);
	});
},'/nanoKONTROL2/mute/7',~korgFromAddr);

OSCdef(\nanoKONTROL2_mute8,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/8", 1);
		~allToVidAddr.sendMsg("/gestures/ctl/show/8", 1);
	},{
		// External control
		~allToMusAddr.sendMsg("/gestures/ctl/show/8", 0);
		~allToVidAddr.sendMsg("/gestures/ctl/show/8", 0);
	});
},'/nanoKONTROL2/mute/8',~korgFromAddr);

// --------------------------------
// ---- CONTROL WEKA PROCESSES ----
// --------------------------------

// ---- Control Weka Articulations ----

OSCdef(\nanoKONTROL2_marker_set,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// ---- Send to Weka ----
		~descToAddr.sendMsg("/wekinator/control/train");
		"Sending OSC: /wekinator/control/train".postln;
		// Send to external devices
		~allToMusAddr.sendMsg("/articulations/ctl/train",1);
		~allToVidAddr.sendMsg("/articulations/ctl/train",1);
		"Sending OSC: /articulations/ctl/train 1".postln;
	},{
		// ---- Send to Weka ----
		~descToAddr.sendMsg("/wekinator/control/cancelTrain");
		"Sending OSC: /wekinator/control/cancelTrain".postln;
		// Send to external devices
		~allToMusAddr.sendMsg("/articulations/ctl/train",0);
		~allToVidAddr.sendMsg("/articulations/ctl/train",0);
		"Sending OSC: /articulations/ctl/train 0".postln;
	});
},'/nanoKONTROL2/marker/set',~korgFromAddr);

OSCdef(\nanoKONTROL2_marker_next,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// ---- Send to Weka ----
		~descToAddr.sendMsg("/wekinator/control/startRunning");
		"Sending OSC: /wekinator/control/startRunning".postln;
		// Send to external devices
		~allToMusAddr.sendMsg("/articulations/ctl/run",1);
		~allToVidAddr.sendMsg("/articulations/ctl/run",1);
		"Sending OSC: /articulations/ctl/run 1".postln;
		//~allToMusAddr.sendMsg("/articulations/values",msg[1],msg[2],msg[3],msg[4],msg[5],msg[6],msg[7],msg[8]);
		//~allToVidAddr.sendMsg("/articulations/values",msg[1],msg[2],msg[3],msg[4],msg[5],msg[6],msg[7],msg[8]);
		"Sending OSC: /articulations/values ... ".postln;
		~artiRunning=1;
	});
},'/nanoKONTROL2/marker/next',~korgFromAddr);

OSCdef(\nanoKONTROL2_marker_last,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// ---- Send to Weka ----
		~descToAddr.sendMsg("/wekinator/control/stopRunning");
		"Sending OSC: /wekinator/control/stopRunning".postln;
		// Send to external devices
		~allToMusAddr.sendMsg("/articulations/ctl/run",0);
		~allToVidAddr.sendMsg("/articulations/ctl/run",0);
		"Sending OSC: /articulations/ctl/run 0".postln;
		~artiRunning=0;
	});
},'/nanoKONTROL2/marker/last',~korgFromAddr);

// ---- Control Weka Gestures ----
OSCdef(\nanoKONTROL2_cycle,{ arg msg, time, addr, recvPort;
		var value = msg[1].asFloat;
	if(value>0,{
		// ---- Send to Weka ----
		~artiToAddr.sendMsg("/wekinator/control/train");
		"Sending OSC: /wekinator/control/train".postln;
		// Send to external devices
		~allToMusAddr.sendMsg("/gestures/ctl/train",1);
		~allToVidAddr.sendMsg("/gestures/ctl/train",1);
		"Sending OSC: /gestures/ctl/train 1".postln;
	},{
		// ---- Send to Weka ----
		~artiToAddr.sendMsg("/wekinator/control/cancelTrain");
		"Sending OSC: /wekinator/control/cancelTrain".postln;
		// Send to external devices
		~allToMusAddr.sendMsg("/gestures/ctl/train",0);
		~allToVidAddr.sendMsg("/gestures/ctl/train",0);
		"Sending OSC: /gestures/ctl/train 0".postln;
	});
},'/nanoKONTROL2/cycle',~korgFromAddr);


OSCdef(\nanoKONTROL2_track_right,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// ---- Send to Weka ----
		~artiToAddr.sendMsg("/wekinator/control/startRunning");
		"Sending OSC: /wekinator/control/startRunning".postln;
		// Send to external devices
		~allToMusAddr.sendMsg("/gestures/ctl/run",1);
		~allToVidAddr.sendMsg("/gestures/ctl/run",1);
		"Sending OSC: /gestures/ctl/run 1".postln;
	});
},'/nanoKONTROL2/track/right',~korgFromAddr);

OSCdef(\nanoKONTROL2_track_left,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		// ---- Send to Weka ----
		~artiToAddr.sendMsg("/wekinator/control/stopRunning");
		"Sending OSC: /wekinator/control/stopRunning".postln;
		// Send to external devices
		~allToMusAddr.sendMsg("/gestures/ctl/run",0);
		~allToVidAddr.sendMsg("/gestures/ctl/run",0);
		"Sending OSC: /gestures/ctl/run 0".postln;
	});
},'/nanoKONTROL2/track/left',~korgFromAddr);

// ----------------------------------------------------------
// ---- SEND RAW DATA FROM WRKINATOR TO MULTIPLE DEVICES ----
// ----------------------------------------------------------
~nanokontrol_fader_values=[0,0,0,0,0,0,0,0];

OSCdef(\articulations,{ arg msg, time, addr, recvPort;
	// Send to Weka
	~artiToAddr.sendMsg("/wek/articulations",msg[1],msg[2],msg[3],msg[4],msg[5],msg[6],msg[7],msg[8]);
	// Send to external devices
	~allToMusAddr.sendMsg("/articulations/values",msg[1],msg[2],msg[3],msg[4],msg[5],msg[6],msg[7],msg[8]);
	~allToVidAddr.sendMsg("/articulations/values",msg[1],msg[2],msg[3],msg[4],msg[5],msg[6],msg[7],msg[8]);
},'/wek/articulations',recvPort:~artiFromPort);

OSCdef(\gestures,{ arg msg, time, addr, recvPort;
	var msgs=Array.fill(~nGestures,0.0);
	~nGestures.do{arg i;
	    // Check and assign backup gesture values
		if(~nanokontrol_fader_values[i]>0,{
			msgs[i]=~nanokontrol_fader_values[i]
		},{
			// Convert from -dB to lin (Note1: Only required for weka data; Note2: Original data is lost
			msgs[i]=msg[i+1];//exp((-1/20)*msg[i+1]);
		});
	};
	// Send to external devices
	~allToMusAddr.sendMsg("/gestures/values",msgs[0],msgs[1],msgs[2],msgs[3],msgs[4],msgs[5],msgs[6],msgs[7]);
	~allToMusAddr.sendMsg("/gestures/max",msgs.maxIndex());
	~allToVidAddr.sendMsg("/gestures/values",msgs[0],msgs[1],msgs[2],msgs[3],msgs[4],msgs[5],msgs[6],msgs[7]);
	~allToVidAddr.sendMsg("/gestures/max",msgs.maxIndex());
	//msgs.postln;
	//msgs.maxIndex().postln;
},'/wek/gestures',recvPort:~gestFromPort);

// -------------------------
// ---- WELCOME MESSAGE ----
// -------------------------
OSCdef(\nanoKONTROL2_welcome,{ arg msg, time, addr, recvPort;
	var value = msg[1].asFloat;
	if(value>0,{
		~allToVidAddr.sendMsg("/welcome", 1);
		~allToVidAddr.sendMsg("/articulations/values",0,0,0,0,0,0,0,0);
		~allToVidAddr.sendMsg("/gestures/values",0,0,0,0,0,0,0,0);
	},{
		~allToVidAddr.sendMsg("/welcome", 0);
	});
},'/nanoKONTROL2/transport/play',~korgFromAddr);


// ----------------------
// ---- FADER BACKUP ----
// ----------------------
OSCdef(\nanoKONTROL2_fader1,{ arg msg, time, addr, recvPort;
	~nanokontrol_fader_values[0]=msg[1].asFloat;
},'/nanoKONTROL2/fader/1',~korgFromAddr);

OSCdef(\nanoKONTROL2_fader2,{ arg msg, time, addr, recvPort;
	~nanokontrol_fader_values[1]=msg[1].asFloat;
},'/nanoKONTROL2/fader/2',~korgFromAddr);

OSCdef(\nanoKONTROL2_fader3,{ arg msg, time, addr, recvPort;
	~nanokontrol_fader_values[2]=msg[1].asFloat;
},'/nanoKONTROL2/fader/3',~korgFromAddr);

OSCdef(\nanoKONTROL2_fader4,{ arg msg, time, addr, recvPort;
	~nanokontrol_fader_values[3]=msg[1].asFloat;
},'/nanoKONTROL2/fader/4',~korgFromAddr);

OSCdef(\nanoKONTROL2_fader5,{ arg msg, time, addr, recvPort;
	~nanokontrol_fader_values[4]=msg[1].asFloat;
},'/nanoKONTROL2/fader/5',~korgFromAddr);

OSCdef(\nanoKONTROL2_fader6,{ arg msg, time, addr, recvPort;
	~nanokontrol_fader_values[5]=msg[1].asFloat;
},'/nanoKONTROL2/fader/6',~korgFromAddr);

OSCdef(\nanoKONTROL2_fader7,{ arg msg, time, addr, recvPort;
	~nanokontrol_fader_values[6]=msg[1].asFloat;
},'/nanoKONTROL2/fader/7',~korgFromAddr);

OSCdef(\nanoKONTROL2_fader8,{ arg msg, time, addr, recvPort;
	~nanokontrol_fader_values[7]=msg[1].asFloat;
},'/nanoKONTROL2/fader/8',~korgFromAddr);
)

Ç

// -----------------
// ---- HELPERS ----
// -----------------
OSCFunc.trace(true);
OSCFunc.trace(false);

