from interfaces import*

miro1 = primary_interface('rob01')
miro1.drive_straight(speed=1)
sleep(2)
miro1.stop_moving()
