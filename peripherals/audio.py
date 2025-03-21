from time import sleep
from robot_hat import Music, TTS

class VehicleAudioSystem:
    """Manages vehicle audio feedback and notifications"""
    
    def __init__(self):
        """Initialize audio subsystems"""
        self.music_player = Music()
        self.speech_synth = TTS()
        
        # Configure audio settings
        self.music_player.music_set_volume(20)
        self.speech_synth.lang("en-US")
        self.bgm_active = False
        
    def play_sound_effect(self, sound_path, threaded=False):
        """Play a sound effect like horn or alert"""
        if threaded:
            self.music_player.sound_play_threading(sound_path)
        else:
            self.music_player.sound_play(sound_path)
        sleep(0.05)  # Small delay for sound to start
            
    def toggle_background_music(self, music_path='../example.mp3'):
        """Toggle background music on/off"""
        self.bgm_active = not self.bgm_active
        if self.bgm_active:
            self.music_player.music_play(music_path)
        else:
            self.music_player.music_stop()
            
    def speak_message(self, message):
        """Convert text to speech for vehicle announcements"""
        self.speech_synth.say(message)
