R_disk = 22.86/2 # cm
height = 1.27 # cm
volume = height*3.1415*(R_disk)**2
density = 7.85 # g/cm^3

weight = volume*density

print(f'Weight: {weight/453.592:02.1f}')

print(f'Diameter: {2*R_disk/25.4:02.2f}')
print(f'Height: {height/25.4:02.2f}')