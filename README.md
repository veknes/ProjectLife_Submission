# Project Life - Monitoring Movement, Heart Beat Rate and Blood Oxygen Level of Elderly People

**Group Name:** Smart Dolphins\
**Owners:** Veknes Benjaman; Saw Jun Chao

**Source Code:** [main.c](/ProjectLife/Core/Src/main.c)\
**Video Demonstration:** [https://youtu.be/MIEMPDX8EPY](https://youtu.be/MIEMPDX8EPY)

### 1.0 Introduction
Monitoring elderly people at home is an important way to ensure their well-being and safety, and to support their independence and quality of life.
There are several reasons why it is important to monitor elderly people at home:

- **Safety**\
Elderly people may be more vulnerable to accidents and injuries, such as falls, due to physical or cognitive impairments. Monitoring can help to identify potential safety hazards and take steps to address them.

- **Health**\
Elderly people may be at higher risk for certain health conditions, such as chronic diseases or infections. Monitoring can help to identify any changes in the elderly person's health and ensure that they receive necessary medical care.

- **Independence**\
Monitoring can help to ensure that elderly people are able to live independently for as long as possible. It can also provide reassurance to elderly people and their families that someone is available to help if needed.

- **Quality of Life**\
Monitoring can help to improve the quality of life for elderly people by ensuring that their needs are met and that they have social and emotional support.

- **Peace of Mind**\
Monitoring can provide peace of mind for family members and caregivers, who can be reassured that the elderly person is safe and well-cared for.

For example, elderly people are at higher risk for severe illness and death from recent COVID-19 for a number of reasons. As people age, their immune systems tend to become weaker, making it more difficult for their bodies to fight off infections. 
In addition, older people are more likely to have underlying health conditions, such as heart disease, diabetes, and lung disease, which can increase their risk of complications from COVID-19 [7].
Therefore, it is essential to monitor them regularly at home.

### 2.0 Use Case
This project consists of developing a STM32 based system for monitoring movement of elderly people at home and alerting family members through push notification if blood oxygen level, heart beat rate are not in normal range or if emergency push button is pressed.

### 3.0 Previous Research / Work
There are a few research done previously similar to this proposed system:

- **"Design of rescue and health monitoring bracelet for the elderly based on STM32"** (2019) [8]\
This study described the development of a bracelet, equipped with an STM32F103 microcontroller and an ST188 heart rate monitor, utilizes Bluetooth technology to communicate with a smartphone app for temperature and heart rate monitoring, as well as a one-button rescue function. The system, which also includes a MLX90614 temperature sensor, allows for body monitoring and emergency assistance. By pressing the help button, the user can quickly call an emergency contact and send their location and heart rate within two seconds.

- **"Design of Health Assistant Bracelet for the Elderly Based on STM32"** (2019) [8]\
This study described the design of a bracelet, equipped with an STM32F103VET6 microcontroller and an ST188 heart rate monitor, uses an InvenSense MPU6050 sensor to track steps and monitor sleep patterns through a connected smartphone app. In addition to providing health monitoring, the bracelet also has a one-button rescue function. By pressing the help button, the user can quickly call an emergency contact and send their location and health information to the emergency contact.

- **"Research and design of elderly health monitoring and anti-fall positioning alarm device based on STM32"** (2021) [8]\
This study described the development of a device, which utilizes an STM32 single chip microcomputer system with a modular design, includes a SIM800 C mobile communication module and a GPS positioning module to provide location and state information in the event of an unexpected fall. It also integrates sensors such as body temperature, heart rate, and step number to monitor the elderly person's health in real time, and sends an alarm if any data are abnormal. Additionally, the device includes a cloud monitoring app based on the E4A platform to allow for real-time monitoring of the elderly person's physical state.
 
### 4.0 System Architecture (Hardware)
The block diagram of the hardware connection is shown in Figure 1. The function and connection type of each components are described in Table 1. The circuit connection on STM32F103VET6 development board and breadboard is shown in Figure 2.

<center><img src="/pictures/1_Hardware_Block_Diagram.png"></center>

Figure 1: Block diagram of hardware connection.

<center><br>Table 1: Function and connection type of components used.</center><br />
<center>

| Component  | Connection Type | Pin Number | Function |
| :-------: | :------: | :------: | :------: |
| 3-Axis Accelerometer (MPU6050) | I2C | PB8 (SCL) PB9 (SDA)| Provides acceleration 3 axes data (the rate of change of velocity) for 3 axis |
| Oximeter and Heart Rate Sensor (MAX30102) | I2C | PB10 (SCL) PB11 (SDA) | Provides blood oxygen level and heart rate data |
| Push Button | INT | PC13 | User input for emergency purpose | 
| ESP8266 | USART | PD5 (TX) PD6 (RX) | Allows internet connectivity and sends data to cloud database (ThingSpeak) |
| USB to TTL | USART | PA9 (TX) PA10 (RX) | UART communication support for debug purpose |

</center>

<center><img src="/pictures/5_Circuit_Connection.jpg"></center>

Figure 2: Circuit connection.


### 5.0 Algorithm (Software)

**5.1 Flowchart of Algorithm**

The flowchart of the algorithm is shown in Figure 3.

<center><img src="/pictures/4_Software_Flowchart.png"></center>

Figure 3: Flowchart of algorithm.

**5.1 Implementing low power mode and RTC wakeup trigger**
- Configured the RTC to wake up the STM32 periodically for every 15 seconds from low power mode (STOP mode) to read sensors and upload collected data to ThingSpeak.
- Configured push button interrupt to wake up STM32 to read sensors and upload collected data to ThingSpeak.
- Calculation for producing Wakeup Counter of 15 seconds
RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16\
Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))\
                 = (16 /(32KHz)) = 0.5ms\
WakeUpCounter = Wakeup Time / Wakeup Time Base\
              = 15s / 0.5ms = 30000 = 0x7530

<center><img src="/pictures/2_Software_SleepMode_Block_Diagram.png"></center>

Figure 4: Block diagram of sleep and wake up mode software algorithm.

**5.2 TinyML model preparation for inferencing**
- 26 data group (3 axes data each) of accelerometer are used to determine if the person are stationary, walking or running.
- Collected [dataset.zip](/ai_model/dataset.zip) for the senario above and trained model with Google Colab to produce [model.h](/ai_model/model.h5) file.
- Utilized X-CUBE-AI expansion package (part of STM32Cube.AI ecosystem). It extends STM32CubeMX capabilities with automatic conversion and optimization of pretrained artificial intelligence algorithms, including neural network and classical machine learning models.
- X-CUBE-AI supports models trained with TensorFlow, Keras, PyTorch, Caffe and others. The model file needs to be in Keras (.h5), TensorFlow Lite (.tflite), or ONNX (.onnx) format.

<center><img src="/pictures/3_Software_TinyML_Block_Diagram.png"></center>

Figure 5: Block diagram of TinyML software algorithm.

**5.3 ThingSpeak**
- Cloud platform to store and monitor all the collected data (heart rate and oximeter sensor, output of AI model and emergency button state).

### 6.0 Results

**6.1 Output of AI model**

- Class 0 (Stationary)
<center><img src="/pictures/results/1_class0_stationary.png"></center>

- Class 1 (Walking)
<center><img src="/pictures/results/2_class1_walking.png"></center>

- Class 2 (Running)
<center><img src="/pictures/results/3_class2_running.png"></center>

- ThingSpeak Movement Monitoring Data
<center><img src="/pictures/results/4_thingspeak_ai_model.png"></center>

**6.2 Output of Emergency Button**
- Putty Output when emergency button is not pressed (1 or True)
<center><img src="/pictures/results/9_emergencybutton_notpressed.png"></center>

- Putty Output when emergency button is pressed (0 or False)
<center><img src="/pictures/results/8_emergencybutton_pressed.png"></center>

- ThingSpeak Emergency Button Data
<center><img src="/pictures/results/7_thingspeak_emergencybutton_data.png"></center>

**6.3 Output of Heart Rate (BPM)**
- Putty Output of Heart Rate Data
<center><img src="/pictures/results/10_hr_data.png"></center>

- ThingSpeak Heart Rate Monitoring Data
<center><img src="/pictures/results/11_thingspeak_hr_data.png"></center>

**6.4 Output of Oxygen Saturation (SP02)**
- Putty Output
<center><img src="/pictures/results/12_spo2_data.png"></center>

- ThingSpeak SP02 Monitoring Data
<center><img src="/pictures/results/13_thingspeak_spo2_data.png"></center>

**6.5 Power consumption during normal and low power (STOP) mode**
- Power consumption when in normal mode = (3.3V)(112.8mA) = 372.24mW
- Power consumption when in low power (STOP) mode = (3.3V)(56.6mA) = 186.78mW
- Power consumption is reduced almost 50% in low power (STOP) mode

<center><img src="/pictures/7_PowerConsumption_NormalMode.jpg"></center>

Figure 6: Current flow when in normal mode.

<center><img src="/pictures/6_PowerConsumption_STOPMode.jpg"></center>

Figure 7: Current flow when in low power (STOP) mode.


### 7.0 Suggestion or Future Improvements
- Modifying the library code to improve the stability of MAX30102 heart rate and SPO2 sensor data.
- Preparing more dataset and retrain the AI model for better accuracy and stability.
- Replacing ThingSpeak to Firebase cloud database as it has triggering method to send notification to mobile application.
- Design mobile application to monitor the sensor data and to receive notification if emergency button is pressed.

## Appendix A: Bill of Materials (BOM)

<center>Table 2: Bill of Materials (BOM).</center><br />


<center>

| Component  | Unit | Cost / Unit (RM) |
| :-------: | :------: | :------: |
| STM32F407VET6 Development Board | 1 | 70 |
| ST-Link V2 USB | 1 | 20 |
| GPU6050 3-Axis Accelerometer | 1 | 10 |
| MAX 30102 Oximeter and Heart Rate Sensor | 1 | 10 |
| Push Button | 1 | 1 | 
| ESP8266 WiFi Serial Transceiver Module | 1 | 10 |
| 18650 Lithium Battery Shield | 1 | 16 |
| 18650 Lithium Battery | 1 | 20 |
|  |  Total Cost (RM) | 157 |

</center>

## References

[1] Board Details and Schematics: [https://stm32-base.org/boards/STM32F407VET6-STM32-F4VE-V2.0.html](https://stm32-base.org/boards/STM32F407VET6-STM32-F4VE-V2.0.html)\
[2] STM32CubeIDE Tutorial:\
[https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf](https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf)\
[https://www.youtube.com/playlist?list=PLnMKNibPkDnFCosVVv98U5dCulE6T3Iy8](https://www.youtube.com/playlist?list=PLnMKNibPkDnFCosVVv98U5dCulE6T3Iy8)\
[3] Motion Sensing Dataset:\
[https://wiki.st.com/stm32mcu/wiki/AI:How_to_perform_motion_sensing_on_STM32L4_IoTnode#Create_an_STM32Cube-AI_application_using_X-CUBE-AI](https://wiki.st.com/stm32mcu/wiki/AI:How_to_perform_motion_sensing_on_STM32L4_IoTnode#Create_an_STM32Cube-AI_application_using_X-CUBE-AI)\
[https://github.com/STMicroelectronics/stm32ai/tree/master/AI_resources/HAR](https://github.com/STMicroelectronics/stm32ai/tree/master/AI_resources/HAR)\
[4] TinyML:\
[https://www.digikey.my/en/maker/projects/tinyml-getting-started-with-stm32-x-cube-ai/f94e1c8bfc1e4b6291d0f672d780d2c0](https://www.digikey.my/en/maker/projects/tinyml-getting-started-with-stm32-x-cube-ai/f94e1c8bfc1e4b6291d0f672d780d2c0)\
[5] ESP8266 STM32 Connection:\
[https://controllerstech.com/esp8266-webserver-using-stm32-hal/]( https://controllerstech.com/esp8266-webserver-using-stm32-hal/)\
[6] STM32 Low Power Mode:\
[https://controllerstech.com/low-power-modes-in-stm32/](https://controllerstech.com/low-power-modes-in-stm32/)\
[https://community.st.com/s/article/how-to-configure-the-rtc-to-wake-up-the-stm32-periodically-from-low-power-modes](https://community.st.com/s/article/how-to-configure-the-rtc-to-wake-up-the-stm32-periodically-from-low-power-modes)\
[7] COVID-19 Inpatient Deaths and Brought-in-Dead Cases in Malaysia:\
[https://www.researchgate.net/publication/361819420_COVID-19_Inpatient_Deaths_and_Brought-in-Dead_Cases_in_Malaysia](https://www.researchgate.net/publication/361819420_COVID-19_Inpatient_Deaths_and_Brought-in-Dead_Cases_in_Malaysia)\
[8] Code library reference:\
[https://github.com/lamik/MAX30102_STM32_HAL/tree/master/Src/MAX30102](https://github.com/lamik/MAX30102_STM32_HAL/tree/master/Src/MAX30102)\
[https://controllerstech.com/data-logger-using-stm32-and-esp8266/](https://controllerstech.com/data-logger-using-stm32-and-esp8266/)\
[9] Previous Research Paper:\
[Design of rescue and health monitoring bracelet for the elderly based on STM32](https://ieeexplore.ieee.org/document/8785440)\
[Design of Health Assistant Bracelet for the Elderly Based on STM32](https://ieeexplore.ieee.org/document/8997765)\
[Research and design of elderly health monitoring and anti-fall positioning alarm device based on STM32](https://ieeexplore.ieee.org/document/9513059)