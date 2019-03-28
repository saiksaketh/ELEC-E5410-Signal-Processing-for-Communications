%RTL_FM_PLL
%
% Simple FM radio receiver using phase-locked loop
% No stereo, no de-emphasis filter
% 
% By R.W.

clear all, close all

% Script to set some essential parameters you need to figure yourself:
% - low-pass filter FLOW and decimator factor NDEC
% - Loop filter and its coefficients
% - Phase detector
% - voltage controlled oscillator
%rw_fmrx_init

%% Radio parameters
%
% FM transmitter
%expFreq = 89.5e6;
% YLE 1
%expFreq = 87.9e6;
% YLE Puhe
expFreq = 103.7e6;
% YLE Radio Suomi
%expFreq = 94e6;

% Front-end sampling rate etc. Change the numbers at will
FESR = 240e3;
nSample = 4092*8;
nFrame = 12e2/8;

NDEC = 6;
fmax = 40e+03;
nyq = fmax/2;

%%
% Task & Explanation: Describe briefly the algorithm used in the filter design function
% The algorithm used is 48th order with frequency magnitude characterstics
% specified by how we set out cutoff frequency,  in the second vector containing the desired magnitude response at each of the points specified in the vector where cutoff frequency is specifies.
% Frequency sampling-based FIR filter design is adopted where FLOW holds
% the values for filter coeffcients which consequently is used to filter
% The FM receiver code is made in such a way, the signal processing blocks are firstly, it performs
% Phase Locked Loop, followed by filtering and finally decimating it.
FLOW = fir2(48, [0 15e3/nyq 17e3/nyq 1], [1 1 0 0]);
 
hSDRrRx = comm.SDRRTLReceiver(...
    'RadioAddress', '0',...
    'CenterFrequency',     expFreq, ...
    'EnableTunerAGC',      true, ...
    'SampleRate',          FESR, ...
    'SamplesPerFrame',     nSample, ...
    'FrequencyCorrection', 70, ...
    'OutputDataType',      'double')
fprintf('\n')

hSpectrumAnalyzer = dsp.SpectrumAnalyzer(...
    'Name',             'Received signal',...
    'Title',            'Received signal', ...
    'SpectrumType',     'Power density',...
    'FrequencySpan',    'Full', ...
    'SampleRate',       FESR, ...
    'YLimits',          [-60,0],...
    'SpectralAverages', 10, ...
    'FrequencySpan',    'Start and stop frequencies', ...
    'StartFrequency',   -50e3, ...
    'StopFrequency',    50e3,...
    'Position',         figposition([50 30 30 40]));

% Check out low-pass filtered signal if necessary
hSA3 = clone(hSpectrumAnalyzer);
set(hSA3,'Name','Filtered signal','Title','filtered signal');

hAudio = audioDeviceWriter(FESR/NDEC,'BufferSize',ceil(nSample*2/NDEC));
getAudioDevices(hAudio);

% Received FM signal
rxfm = zeros(nSample,1);
% Initialization of VCO and, previous phase difference, filter memory
vcoph =0; dphprev=0; lstate=0;


%% Stream Processing                   
if ~isempty(sdrinfo(hSDRrRx.RadioAddress))
	
	fprintf('Receive time %f [s]   \n',nSample/FESR*nFrame)
	
    memo = zeros(1, length(FLOW)-1);
	dphprev=0;
    for iFrame = 1 : nFrame
        rxSig = step(hSDRrRx); 
        rxSig = rxSig - mean(rxSig);  % Remove DC component
 
        % Display received frequency spectrum
        hSpectrumAnalyzer(rxSig);
        
		% Optionally, low-pass filter before the PLL operation
		% Here, low-pass filtering is done after PLL.
		%[rxfilt,memo]=filter(FLOW,1,rxSig,memo);
        %hSA3(rxfilt);
 
		% The loop operates at front-end sampling rate
        for ii = 1:nSample
            % Phase detector 
			dph = rw_phdetector(rxSig(ii), vcoph);
			
            % First-order IIR filter as in the slides
            %rxfm(ii) = rw_loopf(ALPHA,dph,dphprev, lstate);
            rxfm(ii) = rw_loopf(dph,dphprev, lstate);
			dphprev = dph;
			lstate = rxfm(ii);
            
			% "NCO - numerically controlled oscillator"
            vcoph = rw_integrate(rxfm(ii), vcoph);
		end
		
        % Reduce noise level and downsample to Audio
		% Loop filter is low-pass but its stop-band attenuation is poor
        % Filtering done after  PLL, attempt was made to filter it before
        % PLL, 
		[rdfilt,memo]=filter(FLOW,1,rxfm,memo);
		rdec = rdfilt(1:NDEC:end);
		% AGC (automatic gain control). Seems it's not needed
		%rdecnorm = rdec/max(abs(rdec));
        
        % Underrun may occur in the loop
        nUnderrun = hAudio(rdec);
        if nUnderrun > 0
            fprintf('Audio player queue underrun by %d samples.\n',nUnderrun);
        end
    end
else
    warning(message('SDR:sysobjdemos:MainLoop'))
end

%% Release all System objects
release(hSDRrRx); 
clear hSDRrRx
release(hAudio)
%%
%performing functions
function dph  = rw_phdetector(rxsig,vcoph)
%%Strangely, an observation made here is that I was not able to recept
%%anything when I used the model in Slide 14 of PLL lecture, later in
%%co-ordination with Gabriel(other team), I implemented model in Slide 15
%dph = real(rxsig) .* (-sin(vcoph)); According to Slide 14
    dph = imag(rxsig*exp(-1i*vcoph));

end
% First-order IIR filter as in the slides
function rxfm = rw_loopf(dph,dphprev, lstate)

    kp = 0.7 ;
    ki = 0.75;
    rxfm = kp*dph+ki*dph-kp*dphprev+ lstate;
end

function vcoph = rw_integrate(rxfm, vcoph)
    vcoph = rxfm + vcoph;
end 