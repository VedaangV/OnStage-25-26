### setup ###
# navigate to:
# /lib/systemd/system/bluetooth.service.d/nv-bluetooth-service.conf

# edit line:
# ExecStart=/usr/lib/bluetooth/bluetoothd -d --noplugin=audio,a2dp,avrcp
# to:
# ExecStart=/usr/lib/bluetooth/bluetoothd -d

# run:
# sudo apt-get update
# sudo apt-get install pulseaudio-module-bluetooth

pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=4096) 
pygame.mixer.music.set_volume(1.0)

channels = {
    "short_sfx": pygame.mixer.Channel(0),
    "bg_sfx" : pygame.mixer.Channel(1),
    "commentary_sfx" : pygame.mixer.Channel(2)
}

audios = {
    "mining": pygame.mixer.Sound("MINING.mp3")
}

# play audio for ice
def play_sound(channel, name):
    _channel = channels[channel]
    audio = audios[name]
    _channel.play(audio)

while True:

    _input = input()
    play_sound("short_sfx", "mining")

pygame.quit()
