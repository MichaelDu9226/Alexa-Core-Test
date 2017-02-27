# Alexa-Core-Test (the new & awesome version)

This is a client demo for Amazon's Alexa Voice Service(AVS). This pproject is still ongoing.
## Requirements

You will need:

1. **MT7887 HDK**
    - WIFI SOC
    - I2C
    - I2S Slave
2. **WM8978 Audio Codec Module**
    - 16K Sample Rate
    - I2C
    - I2S Slave
3. **Other**
    - Some DuPont Lines(Male to Female)
## User Guide
1. **SDK Download**
    - [LinkIt_SDK_V4.2.0_public](https://github.com/MichaelDu9226/LinkIt_SDK_V4.2.0_public)
    - Extract to LinkIt_SDK_V4.2.0_public
2. **Windows IAR Development Environments**
   
3. **Windows or Linux  GCC Development Environments**
    - [Windoes or Linux GCCguide](https://github.com/MichaelDu9226/LinkIt_SDK_V4.2.0_public/tree/master/doc)
4. **MAC GCC Development Environments**
    - [Mac GCC Download](https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q3-update/+download/gcc-arm-none-eabi-4_8-2014q3-20140805-mac.tar.bz2)
    - Extract gcc-arm-none-eabi-4_8-2014q3-20140805-mac.tar.bz2,and rename folder to gcc-arm-none-eabi
    - Copy gcc-arm-none-eabi to LinkIt_SDK_V4.2.0_public/tools/gcc,and overwrite original folder
5. **Build Demo**
    - [wm8978_record_play_test Demo Download](https://github.com/MichaelDu9226/Alexa-Core-Test)
    - Copy wm8978_record_play_test floder to LinkIt_SDK_V4.2.0_public/project/mt7687_hdk/hal_examples
    - Edit LinkIt_SDK_V4.2.0_public/build.sh
      ````c
      #change Line 15 to:
      export EXTRA_VAR=-j
      ````
    - Enter Terminal,cd XX/XX/LinkIt_SDK_V4.2.0_public/,build
      ````c
      ./build.sh mt7687_hdk wm8978_record_play_test
      ````
7. **Download build image**
    - Ref:LinkIt_7687_HDK_Users_Guide.pdf(LinkIt_SDK_V4.2.0_public/project/mt7687_hdk/hal_examples/doc/HDK/LinkIt_7687_HDK_Users_Guide.pdf)