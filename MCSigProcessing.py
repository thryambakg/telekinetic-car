import numpy as np
from scipy.fft import rfft, rfftfreq
from scipy.signal import butter, sosfiltfilt, welch
from pyOpenBCI import OpenBCICyton
import time
import serial
import math

CYTON_PORT = '/dev/cu.usbserial-DP05IS0W'
ESP_PORT = '/dev/cu.usbserial-0001'
uVoltsConv = (4500000)/24/(2**23-1) #uV/count
accelGConv = 0.002 / (2**4) #G/count
epochChannelData = np.zeros((2, 128))
samplesInEpoch = 0
sampleFreq = 250

#define bandpass filter. we care about theta (4-8 hz) and beta (12-40 hz), so the optimal passband range would be 4-40 hz.
highPass = 4 
lowPass = 40

BANDPASS_SOS = butter(
  N=2, #research suggests that order 2 filter applied backwards and forwards (total order 4, eliminating phase shift) is most optimal for eeg analysis
  Wn=[highPass, lowPass],
  btype='bandpass',
  output='sos', #second-order sections are optimal for filtering because they break down higher-order filtering into numerically stable steps
  fs = sampleFreq
)

board = OpenBCICyton(port=CYTON_PORT, daisy=False, baud=115200, timeout=None, max_packets_skipped=1)
esp = serial.Serial(port=ESP_PORT)

time.sleep(2)

def handleSteering(sample):

  if(sample.aux_data[0] == 0 and sample.aux_data[1] == 0 and sample.aux_data[2] == 0):
    return

  accelX = sample.aux_data[0] * accelGConv #extract acceleration in x direction from sample packet
  accelX = max(-1.0, min(1.0, accelX)) #clamp values to range between -1 and 1 to pass arcsin function

  deg = math.asin(accelX) * (180 / math.pi) #get degree of tilt and convert from radians to degrees

  if (abs(deg) < 5): #create dead zone between -5 and 5 deg
    deg = 0
  elif (deg < -5): #based on car servo sensitivity
    deg = (deg + 5) * 2
  elif (deg > 5):
    deg = (deg - 5) * 2

  esp.write(f"{deg:.0f}\n".encode()) #write degree of tilt to ESP

def throttle(sample):
  #we need to segment into epochs to complete intensity vs time data, apply filters, and then extract features
  global samplesInEpoch
  global epochChannelData
  convertedSample = np.array(sample.channels_data[0:2]) * uVoltsConv #convert sample packet data from byte values to units of microvolts. we only care about PFC for attention, which is why we're only taking fp1 and fp2 readings
  attentionMetric = 0 #this will be what we write to the ESP

  #epoch segmentation - for smooth throttle control, we need 2 Hz updates at least, so roughly 500 ms long updates. Because our Cyton's 
  #sampling rate is 250 Hz, we get 1 sample every 4 ms. 512 is a nice number near 500 divisible by 4, so we can use 512 ms as our epoch length.
  #this gives us 128 samples per epoch (512 ms / 4 ms per 1 sample).
  if (samplesInEpoch == 128): #if we've reached the end of an epoch
    epochChannelData = np.array(epochChannelData)

    filteredSignal = applyBandpass(epochChannelData) #apply filters
    cleanedSignal = removeArtifacts(filteredSignal)

    attentionMetric = extractAttention(cleanedSignal) #get attention metric using theta/beta ratio
    
    epochChannelData = np.zeros((2, 128))
    samplesInEpoch = 0
    esp.write(f"{attentionMetric:.0f}\n".encode()) #write attention metric to ESP
    print(attentionMetric)
  
  epochChannelData[:, samplesInEpoch] = convertedSample
  samplesInEpoch += 1

#variables to play with:
  #window - rectangular, hamming, hanning, bartlett, etc
  #artifact removal
  #feature extraction
def driveCar(sample):
  handleSteering(sample)
  throttle(sample)

def applyBandpass(signal):
  signal = np.asarray(signal) #convert to ndarray if needed
  filtered = sosfiltfilt(BANDPASS_SOS, signal, axis=1) #apply previously defined bandpass filter
  return filtered

def removeArtifacts(signal):
  return signal

def extractAttention(signal):
  global sampleFreq
  thetaMin, thetaMax = 4, 8 #define frequency ranges for theta and beta bands
  betaMin, betaMax = 12, 40

  #welch's method is a super powerful way of computing a power spectral density. the function applies an FFT to convert to the spectral domain,
  #then estimates PSD by dividing the data into overlapping segments and averaging the periodograms to yield a numerically stable and accurate result
  freqsExtracted, transformedPSD = welch(signal, fs=sampleFreq, window='hann', nperseg=128, noverlap=64, axis=1)

  #at this point, we have two important arrays post Fourier Transform:
    #transformedPSD is a spectral domain array that conveys PSD vs frequency bin. 
    #freqsExtracted gives us the Hz values for each frequency bin in transformed.

  thetaFreqs = (freqsExtracted >= thetaMin) & (freqsExtracted <= thetaMax) #create boolean masks for frequency bin ranges that we care about
  betaFreqs = (freqsExtracted >= betaMin) & (freqsExtracted <= betaMax)

  df = freqsExtracted[1] - freqsExtracted[0] #compute unit changes in frequency in our transformedPSD array

  #split transformedPSD into 4 arrays, representing beta and theta range for each channel. integrate each over frequency to get total band power
  #because PSD is given in units of power per hertz. 
  thetaCh1 = np.sum(transformedPSD[0, thetaFreqs]) * df #np.sum integrates when multipled by step size
  betaCh1 = np.sum(transformedPSD[0, betaFreqs]) * df
  thetaCh2 = np.sum(transformedPSD[1, thetaFreqs]) * df
  betaCh2 = np.sum(transformedPSD[1, betaFreqs]) * df

  thetaBandPower = (thetaCh1 + thetaCh2) / 2 #average band powers across Fp1 and Fp2 channels
  betaBandPower = (betaCh1 + betaCh2) / 2

  epsilon = 1e-10 #need this to safeguard against potential division by 0 errors in cases where beta band power is super small
  attentionMetricTBR = thetaBandPower / (betaBandPower + epsilon) #final attention metric is a theta/beta ratio, this is justified by extensive research

  return attentionMetricTBR

try:
  board.start_stream(driveCar)

except KeyboardInterrupt:
  print("\nStopping stream...")
  board.stop_stream()
  esp.close()
  print("Stream stopped cleanly.")