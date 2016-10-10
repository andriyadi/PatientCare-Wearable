# PatientCare-Wearable
Wearable that detects patient comfort, heart beat, and falling movement. 

To install this firmware on ESP9266-based board, all you have to do is open Terminal/Command Prompt and from current directory just type: 

```pio run -t upload```

Of course, you need to install PlatformIO first on your development machine, as this project is created with it. If you don't use [ESPectro board](https://shop.makestro.com/product/espectro-core/), change `platformio.ini` file.

This project is related to the gateway project here: [https://github.com/andriyadi/PatientCare-Gateway](https://github.com/andriyadi/PatientCare-Gateway). 
Make sure to deploy it successfully on your Raspberry Pi.

## Samsung Artik Cloud
Refer to this [writing](https://www.hackster.io/andri/patientcare-for-samsung-artik-cloud-9e68b9) for the details how to deploy software for Wearable, Gateway, and also to configure the cloud.

When you're successfully deploy everything, you'll see on Artik Cloud device log like this:
![Dashboard](https://raw.githubusercontent.com/andriyadi/PatientCare-Wearable/master/assets/Artik/Dashboard.png)

As you can see on that image is that upon the falling is detected by acceleration sensor, FallDetected field will be "true" (green).
