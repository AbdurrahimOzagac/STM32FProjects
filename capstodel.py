import keyboard

print("Harika! Program çalışıyor...")
print("Artık Caps Lock tuşuna bastığında Backspace gibi silecek.")

# Caps Lock tuşunu Backspace'e yönlendiriyoruz
keyboard.remap_key('caps lock', 'backspace')

keyboard.wait('del')