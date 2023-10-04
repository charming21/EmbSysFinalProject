# EmbSysFinalProject
Hello Everyone, thank you for viewing our Spring 2023 final project for Embedded Systems at the University of Utah. Our goal is to design a rover to move from one point to another point, while avoiding obstacles. Using at least 3 of the techniques used in labs(i.e. GPIO,interrupts, timers, UART, I2C, Analog, PID). The purpose of our project is to use our knowledge gained throughtout the semester and implement those skills to build our rover Scorpion.

Team Name: LavenderBlush

Video of our Rover on the move: [Click here for link to rover video](https://photos.app.goo.gl/rmh3rZcpHbNsJV3u7)

Team Members: 

    Brittney Morales
    Saoud Aldowaish
    Zander Bagley

Rover Name: Scorpion 

Project Details: 
    Techniques used for Scorpion:
        - UART (9600 baud rate)
        - Timers (Pulse Width Module)
            - using Timer 2, all four channels
        - GPIO
            - using Port A, B, and C
        - Interrupts 
            - using Interrupts when recieving info via UART
    
    Scorpion Components:
        - Sensors:
            - Scorpion has 5 Ultrasonic sensor, 3 in the front(1 is facing forward, 2 is angled at 45 degrees on both wheels), 1 on the right side, and 1 on the left side.
            - where to buy : https://www.amazon.com/EPLZON-HC-SR04-Ultrasonic-Distance-Arduino/dp/B09PG4HTT1/ref=sr_1_1_sspa?keywords=ultrasonic+sensors&qid=1682463929&sr=8-1-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEzTFFBU1VZWFI3S0NRJmVuY3J5cHRlZElkPUEwNjM4Nzk3MjRNRDQyTTdLRElMSCZlbmNyeXB0ZWRBZElkPUEwNDU3ODQ0MjhWR0c5NVFSM1kwViZ3aWRnZXROYW1lPXNwX2F0ZiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU= 

        - Mecanum Wheels:
            - Scorpion uses mecanum wheels, which allows us to move left and right without dealing with angles of turning.
            - where to buy: https://www.amazon.com/Mecanum-Accessories-OmniDirectional-Components-Rubber/dp/B08VGJG85X/ref=sr_1_16?crid=2TKV5QSHY64F2&keywords=mecanum%2Bwheel&qid=1682463873&s=toys-and-games&sprefix=mecanum%2Bwhe%2Ctoys-and-games%2C252&sr=1-16&th=1 

        - Scorpion Body (Chassi): 
            - we are using a red metal chassis from adafruit.
            - where to buy : https://www.adafruit.com/product/2943 

        - H-Bridge:
            - Scorpion uses a two Dual H-bridges to control all four DC motors. 
            - where to buy : https://www.digikey.com/en/products/detail/sparkfun-electronics/ROB-14450/7915576?utm_adgroup=Evaluation%20and%20Demonstration%20Boards%20and%20Kits&utm_source=google&utm_medium=cpc&utm_campaign=Shopping_Product_Development%20Boards%2C%20Kits%2C%20Programmers&utm_term=&utm_content=Evaluation%20and%20Demonstration%20Boards%20and%20Kits&gclid=CjwKCAjw9J2iBhBPEiwAErwpeUID1lt69zrFmAVwGuZGm8R5AXaUF5Gf5FSMXHxCWttlv_KuX0xVCxoC4XUQAvD_BwE 

        - STM32F0:
            - is a Microcontroller used in Scorpion
            - where to buy : https://www.mouser.com/ProductDetail/511-STM32F072B-DISCO 

        - Motor:
            - Scorpion is using 4 DC motors in a micro servo body that run from 4-6V
            - where to buy: https://www.adafruit.com/product/2941 

        - Power Supply:
            - Scorpion has 2 power supplies
                1. First power supply is a 6V battery pack to power motors only.
                2. Second power supply (power bank with at least 1amp) is to power the STM32 board(where then the STM32 board powers the H-bridge logic and the ultrasonic sensors). 
            - where to buy : https://www.amazon.com/Jex-Electronics-Battery-Holder-Switch/dp/B07DQTNW76/ref=sr_1_5?crid=1R5T2ZHRPWABQ&keywords=6v+power+case&qid=1682462945&s=electronics&sprefix=d6v+power+case%2Celectronics%2C162&sr=1-5

        - Bluetooth Connection:
            - Scorpion uses an HC-05 device which is a USART device.
            - where to buy component : https://www.amazon.com/Wireless-Bluetooth-Receiver-Transceiver-Transmitter/dp/B01MQKX7VP/ref=asc_df_B01MQKX7VP/?tag=hyprod-20&linkCode=df0&hvadid=167146065113&hvpos=&hvnetw=g&hvrand=15314675778965273992&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9029755&hvtargid=pla-362748457327&psc=1  
        
        - Wire Connection:
            - Scorpion has 4 mini breadboards that connects all the components together.
            -where to buy : https://www.amazon.com/DEYUE-breadboard-Set-Prototype-Board/dp/B07LFD4LT6/ref=sr_1_1_sspa?crid=TNBX7YTPDCDT&keywords=breadboard&qid=1682462884&s=electronics&sprefix=breadboard%2Celectronics%2C157&sr=1-1-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUE1TE1YOFJJM0VFUDkmZW5jcnlwdGVkSWQ9QTA5MTEyMjc4VEdENzVXU0JBNUQmZW5jcnlwdGVkQWRJZD1BMDEzNDIyODNGMFRBWU9UWFJSUEwmd2lkZ2V0TmFtZT1zcF9hdGYmYWN0aW9uPWNsaWNrUmVkaXJlY3QmZG9Ob3RMb2dDbGljaz10cnVl 

        - A female USB to mini B USB 
            - to connect the power bank to the STM32F0

        - 3D printed connector between server and mechanum wheel.
            - you will need a total of 4, we recommend to print more to make sure you get a secure fitting.

    Scorpion has 2 modes set through USART:
        - Mode 1: Scorpion will move in the direction the user dictates via USART directions are Forward(W), Back(S), Right(D), Left(A) and Stop(x). Automatically stops when it senses an object in it's path.  

        - Mode 2: Autonomous mode (press 'e' in the putty terminal to put into mode 2), Scorpion moves in forward direction while avoiding objects in it's path. 

    Instruction on how to setup/build your own Scorpion (rover): 
        - Steps:
            - Step 1: Make sure you have all your components with you.
                -Step 1.a: attach servo to chassis with given servo screws.
                -Step 1.b: attach mechanum wheels to servos with 3d printed attachments and use super glue to add to secure the connection.
            - Step 2: Following the wiring Diagram called schematic of Scorpion file, you should be able to wire all your components together. (You can use just wire directly, but we recommend the use of breadboards to make multiple connections.)
            - Step 3: Flash the Scorpian file to your STM32 board.
            - Step 4: Connect the battery pack to your circuit system.
            - Step 5: Connect the power bank to your STM32 board using the usb to mini B cable.
            - Step 6: Press the reset button on the STM32 board
            - Step 7: Connect to your HC-05 Bluetooth from your computer.
            - Step 8: Then setup Putty on your computer
                - Step 8.a: Putty is an application, should be installed on your computer if not, you can install from the web
                - Step 8.b: once installed, open the putty application, click on Serial.
                - Step 8.c: if you know which com your HC-05 is connected to add it to the Serial Line. If not, open up Device manager , look for 'Ports (COM & LPT). click on the drop down, and try the com's listed there. 
            - Step 9: Press the reset button on your STM32 board, and the message : "cmd?" should appear in your putty terminal.

        - We have images on how the rover should like.

    Final Result : 
        - Now going through this steps you should be able to type the letter 'w' for forward, 's' for reverse, 'a' for left, 'd' for right, 'x' for stop, and 'e' for autonomous mode. if you are in the first mode, it will go in that said direction til it is obstructed. It will inform the user which side the blockage is on and the user will then choose which way to go.
        - In it's Autonomous mode (press 'e' in the putty terminal to put into mode 2), Scorpion moves in forward direction while avoiding objects in it's path. It will change direction's itself based on the scenario. 

    Functionality: 
        - Overall functionality, when built correctly and wired correctly Scorpion will avoid obsticles in it's path and move forward with ease. 
    
    Notes: 
        - Attached we have an image of which wheel is which and which timer channel is connected to each wheel for power. This image is called "Wheel Number and Channel"

Git File Description:
	
	========= Final Version ============
	scorpion.c - Final version containing manual UART control with automatic stopping and autonomous driving using the right-hand rule modes (toggle with 'e')

	========= Control ============
	UART control.c - (Testing) UART bluetooth input to robot, able to toggle Forward, Left, Right, Back, and Stop mode with w,a,d,s,x laptop keystrokes 

	========= Motors ============
	PWM Setup.c - (Testing) Initial setup and testing of PWM functionality using TIM2 and TIM3

	working motors direction.c - (Testing) Motor direction (CC/CCW) definitions for each direction of rover according to Mecanum movement, and some quick direction test functions

	new red pcb motor direct and uart.c - (Testing) Combines motor direction functions and UART control to achieve initial motor control for testing on new dual motor driver PCBs

	motor direction.c - (Obsolete) Motor direction (CC/CCW) definitions for each direction of rover according to Mecanum movement, and some quick direction test functions (contains bugs with GPIO definitions) 

	old blue pcb motor direction.c - (Obsolete) Code from early in the project using TIM3 and GPIO pins meant to connect to motor driver PCBs we found to be inadequate

	========= Sensors ============
	ultrasonic testing.c - (Testing) Initial testing and configuration of ultrasonic sensors (used to trim trigger and echo detection and timing) 

	========= Combined Testing ============
	working version without sensors.c - (Testing) Combines UART control, Motor driving, and PWM tuning to adjust for our motors having differing torque 

	working version without autonomous.c - (Testing) Combines UART control, Motor driving, PWM tuning, and Ultrasonic sensing to achieve manual movement with automatic topping if an object is detected in the direction of movement 

