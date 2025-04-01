import numpy as np
from rtlsdr import RtlSdr
import time
from scipy import signal
import threading

class SignalDetector:
    def __init__(self, min_freq=100e6, max_freq=1500e6, threshold=0.5):
        self.min_freq = min_freq
        self.max_freq = max_freq
        self.threshold = threshold
        self.sdr = None
        self.is_running = False
        self.detected_signals = []
        
    def initialize(self):
        """Initialize the RTL-SDR device"""
        try:
            self.sdr = RtlSdr()
            self.sdr.sample_rate = 2.4e6
            self.sdr.gain = 'auto'
            print("RTL-SDR initialized successfully")
        except Exception as e:
            print(f"Error initializing RTL-SDR: {e}")
            raise

    def scan_frequency(self, center_freq):
        """Scan a specific frequency band"""
        try:
            self.sdr.center_freq = center_freq
            samples = self.sdr.read_samples(1024)
            
            # Perform FFT
            fft_data = np.fft.fft(samples)
            freqs = np.fft.fftfreq(len(samples), 1/self.sdr.sample_rate)
            
            # Calculate power spectrum
            power_spectrum = np.abs(fft_data)**2
            
            # Check if signal exceeds threshold
            if np.max(power_spectrum) > self.threshold:
                return {
                    'frequency': center_freq,
                    'power': np.max(power_spectrum),
                    'timestamp': time.time()
                }
            return None
        except Exception as e:
            print(f"Error scanning frequency {center_freq}: {e}")
            return None

    def start_monitoring(self):
        """Start continuous monitoring of the frequency range"""
        self.is_running = True
        self.monitoring_thread = threading.Thread(target=self._monitoring_loop)
        self.monitoring_thread.start()

    def _monitoring_loop(self):
        """Main monitoring loop"""
        while self.is_running:
            current_freq = self.min_freq
            while current_freq <= self.max_freq and self.is_running:
                result = self.scan_frequency(current_freq)
                if result:
                    self.detected_signals.append(result)
                    print(f"Signal detected at {result['frequency']/1e6:.2f} MHz")
                current_freq += self.sdr.sample_rate/2

    def stop_monitoring(self):
        """Stop the monitoring process"""
        self.is_running = False
        if hasattr(self, 'monitoring_thread'):
            self.monitoring_thread.join()
        if self.sdr:
            self.sdr.close()

    def get_detected_signals(self):
        """Return list of detected signals"""
        return self.detected_signals 