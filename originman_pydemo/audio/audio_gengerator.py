#!/usr/bin/env python3
import dashscope
import subprocess
import os
import logging
import re
import tempfile
import argparse

from concurrent.futures import ThreadPoolExecutor

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class TextToSpeech:
    def __init__(self):
        # Configuration parameters
        self.dashscope_api_key = "sk-3d037ba3824c40aba70b2593523ea4d0"
        dashscope.api_key = self.dashscope_api_key
        self.tts_model = "cosyvoice-v1"
        self.voice = "longcheng"
        self.audio_device = "plughw:1,0"
        
        self.tts_executor = ThreadPoolExecutor(max_workers=4)
        self.last_spoken_text = None  # Variable to store the last spoken text
        
        logger.info("Text-to-Speech initialized.")

    def _synthesize_segment(self, text):
        """Synthesize a single text segment and return audio data"""
        synthesizer = dashscope.audio.tts_v2.SpeechSynthesizer(model=self.tts_model, voice=self.voice)
        try:
            logger.info(f"Synthesizing text: '{text}'")
            audio_data = synthesizer.call(text)
            if isinstance(audio_data, bytes):
                return audio_data
            else:
                logger.error(f"Speech synthesis failed to generate valid data: {text}, returned: {audio_data}")
                return None
        except Exception as e:
            logger.error(f"Speech synthesis failed: {e}")
            return None

    def _convert_to_wav(self, mp3_data, output_path=None):
        """Convert MP3 to WAV format and optionally save to a file"""
        try:
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as mp3_file:
                mp3_file.write(mp3_data)
                mp3_path = mp3_file.name
            
            # If output_path is provided, save WAV there; otherwise, use a temp file
            wav_path = output_path if output_path else tempfile.mktemp(suffix='.wav')
            cmd = [
                "ffmpeg",
                "-i", mp3_path,
                "-ac", "2",
                "-ar", "48000",
                "-f", "wav",
                "-acodec", "pcm_s32le",
                "-y",
                wav_path
            ]
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            
            with open(wav_path, 'rb') as f:
                wav_data = f.read()
            
            os.remove(mp3_path)
            # Only remove wav_path if it was a temp file
            if not output_path:
                os.remove(wav_path)
            else:
                logger.info(f"Audio saved to: {output_path}")
            
            return wav_data
        except subprocess.CalledProcessError as e:
            logger.error(f"MP3 to WAV conversion failed: {e.stderr.strip()}")
            return None
        except Exception as e:
            logger.error(f"Error during conversion: {e}")
            return None

    def _play_audio(self, wav_data):
        """Play WAV audio using aplay"""
        if wav_data:
            try:
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as wav_file:
                    wav_file.write(wav_data)
                    wav_path = wav_file.name
                
                cmd = ["aplay", "-D", self.audio_device, wav_path]
                subprocess.run(cmd, check=True, capture_output=True, text=True)
                
                os.remove(wav_path)
            except subprocess.CalledProcessError as e:
                logger.error(f"Audio playback failed: {e.stderr.strip()}")
        else:
            logger.warning("No audio data to play")

    def synthesize_and_play(self, text, output_path=None):
        """Split text into segments, synthesize, play them, and optionally save the last segment"""
        if not text.strip():
            logger.info("Empty text input, skipping synthesis")
            return

        # Split text into sentences
        sentences = re.split(r'[.!?。！？]+', text)
        sentences = [s.strip() + '。' for s in sentences if s.strip()]
        logger.info(f"Text split into {len(sentences)} sentences")

        # Synthesize and play each segment
        futures = [self.tts_executor.submit(self._synthesize_segment, sentence) for sentence in sentences]
        for idx, future in enumerate(futures):
            mp3_data = future.result()
            if mp3_data:
                # Pass output_path only for the last segment if specified
                save_path = output_path if (idx == len(futures) - 1 and output_path) else None
                wav_data = self._convert_to_wav(mp3_data, save_path)
                if wav_data:
                    logger.info(f"Playing segment {idx + 1}")
                    self._play_audio(wav_data)
                    # Update last_spoken_text with the full input text after successful playback
                    if idx == len(futures) - 1:  # Last segment
                        self.last_spoken_text = text

        logger.info("All audio segments played")
        if self.last_spoken_text:
            logger.info(f"Last spoken text updated to: '{self.last_spoken_text}'")

    def run_interactive(self):
        """Interactive mode: accept user input from terminal"""
        logger.info("Entering interactive mode. Enter text and press Enter to generate speech.")
        logger.info("Press Ctrl+C to exit.")
        try:
            while True:
                text = input("Enter text to synthesize (Ctrl+C to exit): ").strip()
                if text:
                    logger.info(f"Received input: '{text}'")
                    self.synthesize_and_play(text)
                else:
                    logger.info("Empty input, please enter some text")
        except KeyboardInterrupt:
            logger.info("User interrupted, shutting down")
            if self.last_spoken_text:
                logger.info(f"Last spoken text was: '{self.last_spoken_text}'")
            else:
                logger.info("No text was spoken during this session")
            self.tts_executor.shutdown(wait=True)

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Generate speech from text using DashScope TTS.")
    parser.add_argument("--txt", type=str, help="Text to synthesize and play (e.g., '我是originman，一个帅气的智能人形机器人')")
    parser.add_argument("--output", type=str, help="Path to save the audio file (e.g., 'output.wav')")
    args = parser.parse_args()

    tts = TextToSpeech()

    if args.txt:
        # Command-line mode: synthesize, play, and optionally save the provided text
        logger.info(f"Received command-line text: '{args.txt}'")
        tts.synthesize_and_play(args.txt, output_path=args.output)
        if tts.last_spoken_text:
            logger.info(f"Last spoken text: '{tts.last_spoken_text}'")
        tts.tts_executor.shutdown(wait=True)
    else:
        # Interactive mode if no text is provided
        tts.run_interactive()

if __name__ == '__main__':
    main()