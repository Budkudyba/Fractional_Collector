# Fractional_Collector
## A Device that collects samples at interval.
A novel spin on fractional collection...Maintaining a Constant volume of the sampled media.  
  
The controller uses the Ramps 1.4 board and Arduino Mega commonly used in 3D printers. 
DRV8825 Stepper Drivers were used.  
  
The sampler takes samples based upon polling an added RTC (real-time clock).  
  
The sampler has two attached parastaltic pumps with stepper motors and a sample tray which is also powered by a stepper motor.  
The sample tray has room for 64 small eppendorf (1.5 mL).  
  
Here's a version of the sampler to show the basic form:  
  
https://a360.co/2QkRquD
  
   
Libraries used:  
https://github.com/adafruit/RTClib  
https://github.com/olikraus/u8g2  
https://github.com/laurb9/StepperDriver  