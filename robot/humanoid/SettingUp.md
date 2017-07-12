# Setting up to work with the Bolide

## Arduino IDE

### Bolide Player Library

#### From the IDE
- open arduino IDE
- In command bar: `sketch->Include Library->Add .Zip Library`
- Navigate to this repo + `ArduinoSetting\libraries` select `BOLIDE_Player` folder. 
- `BOLIDE_Player` should now show up in `sketch->Include Library`

#### Copying files
- open arduino IDE
- In command bar: `File->Preferences` Note the `Sketchbook location`
- Copy from this repo `ArduinoSetting\libraries\BOLIDE_Player` to `<Sketchbook location>\libraries`.
    - If the `libraries` folder does not exist, create it
- Restart Arduino IDE
- `BOLIDE_Player` should now show up in `sketch->Include Library`

#### Using a SymLink (Windows)
- open arduino IDE
- In command bar: `File->Preferences` Note the `Sketchbook location`
- Open Powershell
- navigate to the `Sketchbook location`
- ensure that the `libraries` folder exists, if it does not exist, create it
- `cmd /c mklink /D "<Sketchbook location>\libraries\BOLIDE_Player" "<this repo>\ArduinoSetting\libraries\BOLIDE_Player"`
- Restart Arduino IDE
- `BOLIDE_Player` should now show up in `sketch->Include Library`

### Bolide Board

#### Copying files
- open arduino IDE
- In command bar: `File->Preferences` Note the `Sketchbook location`
- Copy from this repo `ArduinoSetting\hardware\XYZrobot` to `<Sketchbook location>\hardware`.
    - If the `hardware` folder does not exist, create it
- Restart Arduino IDE
- `XYZrobot-BOLIDE` should now show up in `Tools->Board`

#### Using a SymLink (Windows)
- open arduino IDE
- In command bar: `File->Preferences` Note the `Sketchbook location`
- Open Powershell
- navigate to the `Sketchbook location`
- ensure that the `hardware` folder exists, if it does not exist, create it
- `cmd /c mklink /D "<Sketchbook location>\hardware\XYZrobot" "<this repo>\ArduinoSetting\hardware\XYZrobot"`
- Restart Arduino IDE
- `BOLIDE_Player` should now show up in `sketch->Include Library`