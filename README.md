# SCS15/SC15 servo drive with half-duplex UART interface (STM32L432KC)
An STM32 HAL demo of a half-duplex UART communication with the SCS15 Feetech (AKA SC15 Waveshare) servo drive.

![SCS15/SC15 in action](/Assets/Images/scs15_servo_in_action.jpg)

![SCS15/SC15 OLED](/Assets/Images/scs15_servo_oled.jpg)

The SC15 servo from Waveshare seems to be a rebranded SCS15 servo produced by Feetech - you can use manuals and examples from both companies to write your own library. I cannot share here a complete implementation for STM32 because writing the missing part is the lab exercise :innocent: The demo code is written as a blocking one. One of the tasks is to rewrite the library into a non-blocking one (_IT, _DMA, _ReceiveToIdle).

# Missing files?
Don't worry :slightly_smiling_face: Just hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32L4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# Libraries
* OLED: [stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306) (MIT license)

# Software tools
* [Yet Another Terminal :: Serial Communication :: Engineer/Test/Debug](https://sourceforge.net/projects/y-a-terminal/) - check Terminal --> Settings --> Binary setting :hammer_and_wrench:

![Line braking in YAT](/Assets/Images/line_braking_yat.JPG)

![SCS15/SC15 UART messages](/Assets/Images/uart_messages_yat.JPG)

# Exemplary hardware
* [USB to UART converter](https://www.waveshare.com/ch343-usb-uart-board.htm) (Waveshare)

# Docs and examples
* [Feetech manuals EN](https://github.com/synthiam/Behavior_Control_Feetch_Bus_Servos/tree/master/Docs) (Synthiam)
* [Feetech manuals CH](https://www.feetechrc.com/support) (Feetech)
* [Synthiam support page](https://synthiam.com/Support/Skills/Servo/Feetech-Serial-Bus-Servo?id=19570) (Synthiam)
* [SCS15 memory control table](https://synthiam.com/uploads/user/DB763BE15E695777689418BE7364E0A3/vfrmljxv.png) (Synthiam)
* [SC15 17kg Large Torque Programmable Serial Bus Servo](https://www.waveshare.com/sc15-servo.htm) (Waveshare)
* [SC15 Servo](https://www.waveshare.com/wiki/SC15_Servo) (Waveshare)
* [ST/SC serial bus servo control library (Arduino IDE)](https://www.waveshare.com/wiki/Servo_Driver_with_ESP32) (Waveshare)
* [SC series servo sample demo (Arduino IDE)](https://www.waveshare.com/wiki/Servo_Driver_with_ESP32) (Waveshare)
* [How to Communicate using Single Wire || Half Duplex Mode](https://controllerstech.com/stm32-uart-6-communication-using-single-wire/) (ControllersTech)
* [STM32 UART Half-Duplex (Single Wire) Tutorial & Examples](https://deepbluembedded.com/stm32-uart-half-duplex-single-wire-tutorial-example/) (DeepBlueMbedded)
* [How to use 1-wire Protocol to interface DS18B20](https://controllerstech.com/stm32-uart-7-1-wire-protocol/) (ControllersTech)
* [Understanding UART](https://www.youtube.com/watch?v=sTHckUyxwp8) (Rohde Schwarz)

# What next?
Use it/them in one of your projects. For example, build a robotic acrobat similar to [Swinging Monkey](https://www.antonsmindstorms.com/product/robot-inventor-swinging-monkey/) by Antons Mindstorms. And [here](/Assets/Images/sc15_swinging_monkey.jpg) is my take on it. The SCS15/SC15 servos can be bolted to LEGO Technic/Mindstorms bricks without any machining or 3D printing  - probably a coincidence but a pretty handy one :sunglasses:

# Call for action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2024_dzien_otwarty_we/Dzien_Otwarty_WE_2024_Control_Engineering_for_Hobbyists.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them!

> [!WARNING]
> Automation and control engineering - do try this at home :exclamation:

190+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned!
