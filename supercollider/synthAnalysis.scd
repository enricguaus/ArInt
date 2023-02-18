// License: This code is open and free. Feel free to use, modify and redistribute.
// Contact: Please, contact me to suggest improvements, notify errors, share aknowledgements, etc.
// Actions: Boot the server according to the required specifications.
//          Check and adapt "helpers" for specific hardware requirements.
// Description: Boot the server
//              Load synthdefs for: Live input, Rec and Play fron disk, Compute descriptors, Concatenative analysis

(
// -------------------------------
// ---- OPTIONS AND VARIABLES ----
// -------------------------------
s.options.memSize=1024*16;
//s.options.inDevice = "BlackHole 16ch";
//s.options.inDevice = "USB Audio CODEC";
//s.options.inDevice = "Built-in Microph";
//s.options.inDevice = "H1";
//s.options.outDevice = "H1";
//s.options.sampleRate = 48000;
s.meter(2,2);

// ----------------------
// ---- START SERVER ----
// ----------------------

s.waitForBoot(
	// -----------------
	// ---- BUFFERS ----
	// -----------------
	// Place to confiture variables or buffers for synths, if necessary.
	~norm_values=Array.fill(44, { arg i; 0.0 });

    // ------------------
    // ---- SYNTHDEF ----
    // ------------------
	SynthDef(\mic, {arg bufIn=0, bufOut=0, amp=0.9;
		var sig = Mix.new(SoundIn.ar(bufIn,amp));// .tanh;      <-------- Per normalitzar l'entrada?
		Out.ar(bufOut, sig); // Abans rec2disk: Hi havia un !2
	}).add;
	SynthDef(\record2disk, {arg bufIn=0, bufOut=0;
		DiskOut.ar(bufOut, In.ar(bufIn,1));
	}).add;
	SynthDef(\playfromdisk, { arg bufIn=0, bufOut=0, amp=1;
		Out.ar(bufOut, amp*DiskIn.ar(1, bufIn,1));
	}).add;
	SynthDef(\analysis, {arg bufOut=0, bufIn=0, rate=10, fmin=20, fmax=200;
		var descriptors;
		var amp, rms, lou;
		var d_amp, d_rms, d_lou;
		var instFreq, hasFreq, freq, d_freq;
		var onsetAnalysis, beatStatistics;
		var key, chroma;
		var mfcc, centroid, flatness, sensdiss;
		// ---- Input and FFT ----
		var input = Mix.new(In.ar(bufIn));
		var chain = FFT(LocalBuf(2048), input);
		// ---- Amplitude, RMS and Loudness ----
		amp            = MovingAverage.kr(Amplitude.kr(input),5*rate);
		d_amp          = RunningMax.kr(Amplitude.kr(input),Impulse.kr(rate/5))-
		                 RunningMin.kr(Amplitude.kr(input),Impulse.kr(rate/5));
		rms            = MovingAverage.kr(RMS.kr(input),5*rate);
		d_rms          = RunningMax.kr(RMS.kr(input),Impulse.kr(rate/5))-
		                 RunningMin.kr(RMS.kr(input),Impulse.kr(rate/5));
		lou            = MovingAverage.kr(Loudness.kr(chain)/100,5*rate);          // Manual scaling: /100
		d_lou          = RunningMax.kr(Loudness.kr(chain)/100,Impulse.kr(rate/5))- // Manual scaling: /100
		                 RunningMin.kr(Loudness.kr(chain)/100,Impulse.kr(rate/5));
		// ---- Pitch ----
		# instFreq, hasFreq = Tartini.kr(input);
		freq           = MovingAverage.kr(instFreq/1000,5*rate);                  // Manual scaling: /1000
		d_freq         = RunningMax.kr(instFreq/1000,Impulse.kr(rate/5))-         // Manual scaling: /1000
		                 RunningMin.kr(instFreq/1000,Impulse.kr(rate/5));
		// ---- Rhythm ----
		onsetAnalysis  = OnsetStatistics.kr(Onsets.kr(chain,0.25),2.0)/10;        // Manual scaling: /10
		beatStatistics = BeatStatistics.kr(chain)/10;                             // Manual scaling: /10
		// ---- Chroma ----
		key            = KeyTrack.kr(chain, keydecay: 2.0, chromaleak: 0.25)/100; // Manual scaling: /100
		chroma         = MovingAverage.kr(Chromagram.kr(chain),5*rate);
		// ---- Timbre ----
		centroid = MovingAverage.kr(SpecCentroid.kr(chain)/10000,5*rate);          // Manual scaling: /10000
		flatness = MovingAverage.kr(SpecFlatness.kr(chain),5*rate);
		sensdiss = MovingAverage.kr(SensoryDissonance.kr(chain),5*rate);
		mfcc     = MovingAverage.kr(MFCC.kr(chain),5*rate); mfcc[0]=mfcc[0]/2;    // Manual scaling: /2
		// ---- Send data ----
		descriptors=[amp,d_amp,rms,d_rms,lou,d_lou,freq,d_freq,
			onsetAnalysis[0],onsetAnalysis[1],onsetAnalysis[2],
			beatStatistics[0],beatStatistics[1], beatStatistics[2],beatStatistics[3],
			key,chroma[0],chroma[1],chroma[2],chroma[3],chroma[4],chroma[5],chroma[6],chroma[7],
			chroma[8],chroma[9],chroma[10],chroma[11],
			centroid,flatness,sensdiss,mfcc[0],mfcc[1],mfcc[2],mfcc[3],mfcc[4],mfcc[5],mfcc[6],mfcc[7],
			mfcc[8],mfcc[9],mfcc[10],mfcc[11],mfcc[12]];
		Out.kr(bufOut,descriptors);
		Out.ar(0,input!2);
	}).add;

	SynthDef(\concat, {arg bufCtl=0, bufIn=0, bufOut=0, dry=0, lag=0.5, amp=1, freeze=0;
		var control = Mix.new(SoundIn.ar(bufCtl));
		var input= PlayBuf.ar(1,bufIn,BufRateScale.kr(bufIn), loop:1);
		var concat= Concat2.ar(control,input,5.0,5.0,5.0,0.25,freeze,0.2,0.2,0.2,0.2,0.5,0.001);
		var out = XFade2.ar(input,concat,Lag2.kr(dry,lag).linlin(0,1,1,-1),2*Lag2.kr(amp,lag));
		var rev=FreeVerb.ar(out,0.7,0.7,0.01,mul:2);
		Out.ar(bufOut,Pan2.ar(rev,pos: LFNoise2.kr(2)));
	}).add;
);
)

(
// ----------------------
// ---- CLOSE SERVER ----
// ----------------------
s.freeAll;
s.quit;
Server.killAll;
)

