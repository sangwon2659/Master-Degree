# Matlab Installation
#### Download matlab installation file (R2021a for Linux)
https://au.mathworks.com/downloads/
#### Unzip and execute the following
```
cd Downloads
sudo sh install
```
#### Install to the preferred location
```
/usr/local/MATLAB
```
***
#### Creating desktop entry
```
sudo wget http://upload.wikimedia.org/wikipedia/commons/2/21/Matlab_Logo.png -O /usr/share/icons/matlab.png
sudo touch /usr/share/applications/matlab.desktop
sudo gedit /usr/share/applications/matlab.desktop
```
Then paste the following into the file
```
#!/usr/bin/env xdg-open
[Desktop Entry]
Type=Application
Icon=/usr/share/icons/matlab.png
Name=MATLAB R2017b
Comment=Start MATLAB - The Language of Technical Computing
Exec=matlab -desktop -useStartupFolderPref 
Categories=Development;
```
